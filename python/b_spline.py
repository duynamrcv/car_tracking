import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate


def approximate_b_spline_path(x: list,
                              y: list,
                              n_path_points: int,
                              degree: int = 3,
                              s=None,
                              ) -> tuple:
    """
    Approximate points with a B-Spline path

    Parameters
    ----------
    x : array_like
        x position list of approximated points
    y : array_like
        y position list of approximated points
    n_path_points : int
        number of path points
    degree : int, optional
        B Spline curve degree. Must be 2<= k <= 5. Default: 3.
    s : int, optional
        smoothing parameter. If this value is bigger, the path will be
        smoother, but it will be less accurate. If this value is smaller,
        the path will be more accurate, but it will be less smooth.
        When `s` is 0, it is equivalent to the interpolation. Default is None,
        in this case `s` will be `len(x)`.

    Returns
    -------
    x : array
        x positions of the result path
    y : array
        y positions of the result path
    heading : array
        heading of the result path
    curvature : array
        curvature of the result path

    """
    distances = _calc_distance_vector(x, y)

    spl_i_x = interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = interpolate.UnivariateSpline(distances, y, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    return _evaluate_spline(sampled, spl_i_x, spl_i_y)


def interpolate_b_spline_path(x, y,
                              n_path_points: int,
                              degree: int = 3) -> tuple:
    """
    Interpolate x-y points with a B-Spline path

    Parameters
    ----------
    x : array_like
        x positions of interpolated points
    y : array_like
        y positions of interpolated points
    n_path_points : int
        number of path points
    degree : int, optional
        B-Spline degree. Must be 2<= k <= 5. Default: 3

    Returns
    -------
    x : array
        x positions of the result path
    y : array
        y positions of the result path
    heading : array
        heading of the result path
    curvature : array
        curvature of the result path

    """
    return approximate_b_spline_path(x, y, n_path_points, degree, s=0.0)


def _calc_distance_vector(x, y):
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]
    return distances


def _evaluate_spline(sampled, spl_i_x, spl_i_y):
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    dx = spl_i_x.derivative(1)(sampled)
    dy = spl_i_y.derivative(1)(sampled)
    heading = np.arctan2(dy, dx)
    ddx = spl_i_x.derivative(2)(sampled)
    ddy = spl_i_y.derivative(2)(sampled)
    curvature = (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 2.0 / 3.0)
    return np.array(x), y, heading, curvature,

def load_csv(file_name):
    df = pd.read_csv(file_name, sep=' ')
    df = np.array(df)
    
    timestamp = df[:,0]
    x = df[:100,1]
    y = df[:100,2]
    # z = df[:,3]

    # theta = []
    # for i in range(df.shape[0]):
    #     _, _, yaw = quaternion_to_euler(df[i,4:8])
    #     theta.append(yaw)
    # gear = df[:,8]
    return timestamp, x, y

if __name__ == '__main__':
    # way points
    file_name = "data/basement1.txt"
    timestamp, way_point_x, way_point_y = load_csv(file_name)
    n_course_point = 200  # sampling number

    plt.subplots()
    rax, ray, heading, curvature = approximate_b_spline_path(
        way_point_x, way_point_y, n_course_point, degree=5, s=0.02)
    plt.plot(rax, ray, '-or', label="Approximated B-Spline path")
    # plot_curvature(rax, ray, heading, curvature)

    plt.title("B-Spline approximation")
    plt.plot(way_point_x, way_point_y, 'xg', label="way points")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()
