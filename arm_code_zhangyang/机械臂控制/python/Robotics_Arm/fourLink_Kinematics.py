import warnings
import numpy as np
from spatialmath.base import *
from spatialmath import *
import matplotlib.pyplot as plt
from roboticstoolbox import *
from collections import namedtuple

np.set_printoptions(threshold=np.inf)


# noinspection PyShadowingNames,PyUnboundLocalVariable
class four_Link(object):
    def __init__(self, q=None, link_ps=None):
        self.__ZERO = 0
        self.PI = np.pi
        self.todeg = 180 / self.PI
        self.torad = self.PI / 180
        self.__theta = np.array([0, ] * 16) if q is None else q
        self.link0, self.link1, self.link2, self.link3, self.link4 = [0.8, 0.2, 0.6, 0.15, 0.081] \
            if link_ps is None else link_ps
        self._rx_90, self._rx_180, self._ry_180 = SE3(trotx(self.PI / 2)), SE3(trotx(-self.PI)), SE3(troty(self.PI))
        self.__world = SE3(transl(0, 0, 0))
        self.__base = self.__world @ SE3(np.linalg.inv(self._rx_90))
        self.obstacles = []

    def fkine(self, q=None):
        self.__theta = q if q is not None else self.__theta
        rx_dif_theta = SE3(trotz(self.__theta[0] + self.__theta[1], 'deg'))
        rot_joints = SE3([trotz(self.__theta[i], 'deg') for i in range(len(self.__theta))])
        P0 = SE3(transl(self.__ZERO, self.__ZERO, self.link0))
        P1 = SE3(transl(self.link2, self.__ZERO, self.__ZERO))
        P2 = SE3(transl(self.link1, self.__ZERO, self.__ZERO))
        P3 = SE3(transl(self.link3, self.__ZERO, self.__ZERO))
        P4 = SE3(transl(self.link4, self.__ZERO, self.__ZERO))
        pose1 = self.__base @ rot_joints[0] @ self._rx_90 @ P0
        temp = pose1 @ self._rx_90 @ rx_dif_theta
        pose2 = temp @ P1
        temp1 = pose2 @ self._rx_180
        pose3 = temp @ SE3(transl(-self.link1, 0, 0))
        pose4 = self.__base @ self._ry_180 @ rot_joints[1] @ P2
        pose5 = temp1 @ rot_joints[2] @ P3
        temp2 = pose5 @ self._rx_90
        pose6 = temp2 @ rot_joints[3] @ P3
        temp3 = pose6 @ SE3(np.linalg.inv(self._rx_90))
        pose7 = temp3 @ rot_joints[4] @ P3
        temp4 = pose7 @ self._rx_90
        pose8 = temp4 @ rot_joints[5] @ P3
        self._pose1_18 = [pose1, pose4, pose3, pose2, pose5, pose6, pose7, pose8]
        temp_15 = [temp, temp1, temp2, temp3, temp4, pose8]
        for i in range(6, 16):
            pose = temp_15[-1] @ rot_joints[i] @ P4
            self._pose1_18.append(pose)
            temp6 = self._pose1_18[-1] @ self._rx_90 if i % 2 == 0 else self._pose1_18[-1] @ SE3(
                np.linalg.inv(self._rx_90))
            temp_15.append(temp6)
        return temp_15[-1]

    @property
    def __get_joint(self):
        points = np.r_[np.array([[0, 0, 0]]), np.array([matr.A[:3, 3] for matr in self._pose1_18])]
        return points

    def frame_teach(self, q=None, view_limit=None, add_obstacle=False, **kwargs):
        self.__theta = q if q is not None else self.__theta
        self.fkine(self.__theta)
        info = self.__get_joint
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        if kwargs.get('addPipe') is True:
            pipe = self.__pipeData(kwargs.get('T1', troty(self.PI / 2)), kwargs.get('center', (1.3, 0, 0.8)),
                                   kwargs.get('radius', 0.2), kwargs.get('height', 0.5))
            ax.plot_surface(pipe[0], pipe[1], pipe[2], cmap=kwargs.get('cmap', 'rainbow'))
        if add_obstacle:
            for obs_user in self.obstacles:
                if obs_user.species == 'box':
                    ax.voxels(obs_user.data[0], obs_user.data[1], obs_user.data[2], obs_user.data[3], color=obs_user.cm)
                elif obs_user.species == 'sphere':
                    ax.plot_surface(obs_user.data[0], obs_user.data[1], obs_user.data[2], cmap=obs_user.cm)
        # exit()
        self.__darw(ax, info, view_limit)
        plt.show()

    @property
    def __fourLink_(self):
        DHs = [RevoluteDH(qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(alpha=self.PI / 2, qlim=[0, 0]),
               RevoluteDH(d=self.link0, alpha=self.PI / 2, qlim=[0, 0]),
               RevoluteDH(a=self.link2, alpha=-self.PI, qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(a=self.link3, alpha=self.PI / 2, qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(a=self.link3, alpha=-self.PI / 2, qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(a=self.link3, alpha=self.PI / 2, qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(a=self.link3, qlim=[-80 * self.torad, 80 * self.torad]),
               RevoluteDH(a=self.link4, alpha=self.PI / 2, qlim=[-80 * self.torad, 80 * self.torad])]
        for i in range(1, 10):
            DHs.append(
                RevoluteDH(a=self.link4, alpha=(self.PI / 2) * (-1) ** i, qlim=[-80 * self.torad, 80 * self.torad]))
        fourLink_ = DHRobot(DHs)
        fourLink_.base = fourLink_.base @ SE3(np.linalg.inv(self._rx_90))
        return fourLink_

    def ikine(self, T_, q0=None, step=150, method='L-BFGS-B', plot=False, **kwargs):
        assert isinstance(T_, SE3) | isinstance(T_, np.ndarray) | isinstance(T_, np.matrix)
        self.__theta = q0 if q0 is not None else self.__theta
        theta__ = np.array([self.__theta[0], self.__ZERO, self.__ZERO, self.__theta[0] + self.__theta[1]])
        theta = np.r_[theta__, self.__theta[2:]] * self.torad
        fourLink = self.__fourLink_
        # fourLink.teach()
        # exit()
        T0 = fourLink.fkine(theta)
        Ts = ctraj(T0, T_, step)
        sol_temp = [theta]
        for T in Ts:
            sol = fourLink.ikine_min(T, sol_temp[-1], qlim=True, method=method)
            sol_temp.append(sol.q)
        sol_temp = np.array(sol_temp)
        sol_temp = np.array(sol_temp) * self.todeg
        theta1 = np.array(sol_temp[:, 3] - sol_temp[:, 0]).reshape(-1, 1)
        sol_ikine = np.c_[sol_temp[:, 0], theta1, sol_temp[:, 4:]]
        T_end = self.fkine(sol_ikine[-1])
        T_end = np.array(T_end)
        T_end[np.abs(T_end) < 0.000001] = 0
        T_end = SE3(T_end)
        if (T_end == T_) & plot:
            self.frame_plot(sol_ikine, view_limit=kwargs.get('view_limit'), hold=kwargs.get('hold', False))
            return sol_ikine, True
        else:
            if (not plot) & (T_end == T_):
                return sol_ikine, True
            elif not (T_end == T_) and plot:
                warnings.warn("Target pose is not available!")
                self.frame_plot(sol_ikine, view_limit=kwargs.get('view_limit'), hold=kwargs.get('hold', False))
                return sol_ikine, False
            warnings.warn("Target pose is not available!")
            return sol_ikine, False

    def frame_plot(self, solq: np.ndarray, lw=4.0, view_limit=None, hold=False, **kwargs):
        if len(solq) == 1:
            self.frame_teach(solq[0])
        symbol = kwargs.get('addPipe', False)
        if symbol:
            pipe = self.__pipeData(kwargs.get('T1', troty(self.PI / 2)), kwargs.get('center', (1.3, 0, 0.8)),
                                   kwargs.get('radius', 0.2), kwargs.get('height', 0.5))
        if kwargs.get('add_obstacle') is True:
            ob = self.obstacles
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        plt.ion()
        for q in solq:
            self.fkine(q)
            info = self.__get_joint
            plt.cla()
            if symbol:
                ax.plot_surface(pipe[0], pipe[1], pipe[2], cmap=kwargs.get('cmap', 'rainbow'))
            for obs_user in ob:
                if obs_user.species == 'box':
                    ax.voxels(obs_user.data[0], obs_user.data[1], obs_user.data[2], obs_user.data[3],
                              color=obs_user.cm)
                elif obs_user.species == 'sphere':
                    ax.plot_surface(obs_user.data[0], obs_user.data[1], obs_user.data[2], cmap=obs_user.cm)
            self.__darw(ax, info, view_limit, lw)
            plt.pause(0.005)
        plt.ioff()
        if hold is True:
            plt.show()

    @classmethod
    def __darw(cls, ax: plt.axes, info: np.ndarray, view_limit: None | list, lw=4.0):
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        if view_limit is not None:
            ax.set_xlim(view_limit[:2])
            ax.set_ylim(view_limit[2:4])
            ax.set_zlim(view_limit[4:])
        else:
            ax.set_xlim([-0.5, 1.5])
            ax.set_ylim([-1., 1.])
            ax.set_zlim([-0.5, 1.5])
        ax.set_box_aspect([1, 1, 1])
        link_0 = [[info[0, 0], info[1, 0]], [info[0, 1], info[1, 1]], [info[0, 2], info[1, 2]]]
        link_1 = [[info[0, 0], info[2, 0]], [info[0, 1], info[2, 1]], [info[0, 2], info[2, 2]]]
        links = [link_0, link_1]
        for i in range(2, 18):
            links.append([[info[i, 0], info[i + 1, 0]], [info[i, 1], info[i + 1, 1]], [info[i, 2], info[i + 1, 2]]])
        links = np.array(links)
        ax.plot(links[0][0], links[0][1], links[0][2], c='#FF4500', linewidth=lw)
        ax.plot(links[1][0], links[1][1], links[1][2], c='mediumslateblue', linewidth=lw)  # #7B68EE
        ax.plot(links[3][0], links[3][1], links[3][2], c='#FF4500', linewidth=lw)
        ax.plot(links[2][0], links[2][1], links[2][2], c='mediumslateblue', linewidth=lw)
        j = 4
        for i in range((len(links) - 4) // 2):
            if j < 8:
                ax.plot(links[j][0], links[j][1], links[j][2], c='c', linewidth=lw)
                ax.plot(links[j + 1][0], links[j + 1][1], links[j + 1][2], c='sienna', linewidth=lw)
                j += 2
            else:
                ax.plot(links[j][0], links[j][1], links[j][2], c='dodgerblue', linewidth=lw)
                ax.plot(links[j + 1][0], links[j + 1][1], links[j + 1][2], c='m', linewidth=lw)
                j += 2

    def draw_pipe(self, T1=np.eye(4), center=(0, 0, 0), radius=1, height=3, view_limit=None, **kwargs):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        if view_limit is not None:
            ax.set_xlim(view_limit[:2])
            ax.set_ylim(view_limit[2:4])
            ax.set_zlim(view_limit[4:])
        else:
            ax.set_xlim([-2, 2])
            ax.set_ylim([-2, 2])
            ax.set_zlim([0, 4])
        ax.set_box_aspect([1, 1, 1])
        pipe = self.__pipeData(T1, center, radius, height, hsplit=kwargs.get('hsplit', 100),
                               angle_split=kwargs.get('angle_split', 50))
        print(pipe[0].shape, pipe[1].shape, pipe[2].shape, sep='\n')
        pipe_d = np.r_[pipe[0], pipe[1], pipe[2]]
        print(pipe_d.shape)
        # np.savetxt()
        exit()
        ax.plot_surface(pipe[0], pipe[1], pipe[2], cmap=kwargs.get('cmap', 'rainbow'))
        plt.show()

    @classmethod
    def __pipeData(cls, _T1: np.ndarray, center: tuple, radius: float, height: float, hsplit=50, angle_split=50):
        u = np.linspace(0, 2 * np.pi, angle_split)
        h = np.linspace(0, height, hsplit)
        x = np.outer(np.sin(u), radius * np.ones(len(h)))
        y = np.outer(np.cos(u), radius * np.ones(len(h)))
        z = np.outer(np.ones(len(u)), h)
        xy_coord = np.c_[x[:, 0], y[:, 0]]
        one = np.array([1, ] * angle_split).reshape(-1, 1)
        pipe = np.array([np.c_[xy_coord, z_coord, one] for z_coord in z.T])
        for i in range(len(pipe)):
            pipe[i] = np.matmul(np.array(_T1), pipe[i].T).T
        pipe_X, pipe_Y, pipe_Z = center[0] + pipe[:, :, 0].T, center[1] + pipe[:, :, 1].T, center[2] + pipe[:, :, 2].T
        return pipe_X, pipe_Y, pipe_Z

    def obstacle(self, Category, **kwargs):
        user_obstace = namedtuple("Obstace", "species, center_obs, radius_obs, cm, data")
        center = kwargs.get('center', (0, 0, 0))
        if Category == 'box':
            _x, _y, _z = np.indices((2, 2, 2))
            x = center[0] + _x * kwargs.get('x', 1)
            y = center[1] + _y * kwargs.get('y', 1)
            z = center[2] + _z * kwargs.get('z', 1)
            filled = np.ones((1, 1, 1))
            solution = user_obstace('box', center, None, kwargs.get('cm'), [x, y, z, filled])
            self.obstacles.append(solution)
            return x, y, z, filled
        elif Category == 'sphere':
            sphereData = sphere(radius=kwargs.get('radius', 1), centre=center)
            '''
            'Accent', 'Accent_r', 'Blues', 'Blues_r', 'BrBG', 'BrBG_r', 'BuGn', 'BuGn_r', 'BuPu', 'BuPu_r', 'CMRmap', 'CMRmap_r', 'Dark2', 'Dark2_r', 'GnBu', 'GnBu_r', 'Greens', 'Greens_r', 'Greys', 'Greys_r', 'OrRd', 'OrRd_r', 'Oranges', 'Oranges_r', 'PRGn', 'PRGn_r', 'Paired', 'Paired_r', 'Pastel1', 'Pastel1_r', 'Pastel2', 'Pastel2_r', 'PiYG', 'PiYG_r', 'PuBu', 'PuBuGn', 'PuBuGn_r', 'PuBu_r', 'PuOr', 'PuOr_r', 'PuRd', 'PuRd_r', 'Purples', 'Purples_r', 'RdBu', 'RdBu_r', 'RdGy', 'RdGy_r', 'RdPu', 'RdPu_r', 'RdYlBu', 'RdYlBu_r', 'RdYlGn', 'RdYlGn_r', 'Reds', 'Reds_r', 'Set1', 'Set1_r', 'Set2', 'Set2_r', 'Set3', 'Set3_r', 'Spectral', 'Spectral_r', 'Wistia', 'Wistia_r', 'YlGn', 'YlGnBu', 'YlGnBu_r', 'YlGn_r', 'YlOrBr', 'YlOrBr_r', 'YlOrRd', 'YlOrRd_r', 'afmhot', 'afmhot_r', 'autumn', 'autumn_r', 'binary', 'binary_r', 'bone', 'bone_r', 'brg', 'brg_r', 'bwr', 'bwr_r', 'cividis', 'cividis_r', 'cool', 'cool_r', 'coolwarm', 'coolwarm_r', 'copper', 'copper_r', 'cubehelix', 'cubehelix_r', 'flag', 'flag_r', 'gist_earth', 'gist_earth_r', 'gist_gray', 'gist_gray_r', 'gist_heat', 'gist_heat_r', 'gist_ncar', 'gist_ncar_r', 'gist_rainbow', 'gist_rainbow_r', 'gist_stern', 'gist_stern_r', 'gist_yarg', 'gist_yarg_r', 'gnuplot', 'gnuplot2', 'gnuplot2_r', 'gnuplot_r', 'gray', 'gray_r', 'hot', 'hot_r', 'hsv', 'hsv_r', 'inferno', 'inferno_r', 'jet', 'jet_r', 'magma', 'magma_r', 'nipy_spectral', 'nipy_spectral_r', 'ocean', 'ocean_r', 'pink', 'pink_r', 'plasma', 'plasma_r', 'prism', 'prism_r', 'rainbow', 'rainbow_r', 'seismic', 'seismic_r', 'spring', 'spring_r', 'summer', 'summer_r', 'tab10', 'tab10_r', 'tab20', 'tab20_r', 'tab20b', 'tab20b_r', 'tab20c', 'tab20c_r', 'terrain', 'terrain_r', 'turbo', 'turbo_r', 'twilight', 'twilight_r', 'twilight_shifted', 'twilight_shifted_r', 'viridis', 'viridis_r', 'winter', 'winter_r'
            '''
            solution = user_obstace('sphere', center, kwargs.get('radius', 1), kwargs.get('cm', 'winter'), sphereData)
            self.obstacles.append(solution)
            return sphereData


if __name__ == '__main__':
    fl = four_Link()
    box0 = fl.obstacle(Category='box', center=(0.8, 0.3, 0.4), x=0.5, y=0.5, z=0.5, cm='sienna')
    box1 = fl.obstacle(Category='box', center=(1.2, -0.5, 0.4), x=0.5, y=0.3, z=0.3, cm='darkolivegreen')
    sphere0 = fl.obstacle(Category='sphere', center=(0.7, -0.4, 1), radius=0.3, )

    theta_test = [-80., 81.06933937, 44.0917309, 12.13745438, 40.47644288, 23.56533703, 16.39976207, -7.8882776,
                  8.71706159, 4.03461056, -5.56892614, 9.29348383, -21.21273515, 5.8255818, -31.4682967, -4.78495544]

    # fl.frame_teach(addPipe=True)
    fl.draw_pipe()
    solq, success = fl.ikine(SE3(transl(2.5, 0.3, 0.8)), theta_test)
    if success:
        fl.frame_plot(solq, hold=False, addPipe=True, add_obstacle=False)
