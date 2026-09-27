from __future__ import annotations

import math

Point3 = tuple[float, float, float]
RobotPose4 = tuple[float, float, float, float]


def solve_camera_to_robot_matrix(camera_points: list[Point3], robot_points: list[Point3]) -> list[float] | None:
    if len(camera_points) < 3 or len(robot_points) < 3:
        return None
    camera_center = centroid(camera_points)
    robot_center = centroid(robot_points)
    camera_delta = [subtract(point, camera_center) for point in camera_points]
    robot_delta = [subtract(point, robot_center) for point in robot_points]
    if not has_spatial_variation(camera_delta):
        return None
    rotation = horn_rotation(camera_delta, robot_delta)
    translation = subtract(robot_center, rotate(rotation, camera_center))
    return [
        rotation[0][0],
        rotation[0][1],
        rotation[0][2],
        translation[0],
        rotation[1][0],
        rotation[1][1],
        rotation[1][2],
        translation[1],
        rotation[2][0],
        rotation[2][1],
        rotation[2][2],
        translation[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def calibration_errors(matrix: list[float], pairs: list[tuple[Point3, RobotPose4]]) -> list[float]:
    return [point_distance(transform_camera_point(matrix, camera_point), robot_xyz(robot_point)) for camera_point, robot_point in pairs]


def transform_camera_point(matrix: list[float], point: Point3) -> Point3:
    return (
        matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
        matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
        matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
    )


def robot_xyz(point: RobotPose4) -> Point3:
    return point[0], point[1], point[2]


def point_distance(left: Point3, right: Point3) -> float:
    dx = left[0] - right[0]
    dy = left[1] - right[1]
    dz = left[2] - right[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def centroid(points: list[Point3]) -> Point3:
    count = float(len(points))
    return (
        sum(point[0] for point in points) / count,
        sum(point[1] for point in points) / count,
        sum(point[2] for point in points) / count,
    )


def subtract(left: Point3, right: Point3) -> Point3:
    return left[0] - right[0], left[1] - right[1], left[2] - right[2]


def rotate(rotation: list[list[float]], point: Point3) -> Point3:
    return (
        rotation[0][0] * point[0] + rotation[0][1] * point[1] + rotation[0][2] * point[2],
        rotation[1][0] * point[0] + rotation[1][1] * point[1] + rotation[1][2] * point[2],
        rotation[2][0] * point[0] + rotation[2][1] * point[1] + rotation[2][2] * point[2],
    )


def has_spatial_variation(points: list[Point3]) -> bool:
    for first in range(len(points)):
        for second in range(first + 1, len(points)):
            for third in range(second + 1, len(points)):
                a = points[first]
                b = points[second]
                c = points[third]
                ab = subtract(b, a)
                ac = subtract(c, a)
                cross = (
                    ab[1] * ac[2] - ab[2] * ac[1],
                    ab[2] * ac[0] - ab[0] * ac[2],
                    ab[0] * ac[1] - ab[1] * ac[0],
                )
                if cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2] > 1e-6:
                    return True
    return False


def horn_rotation(camera_delta: list[Point3], robot_delta: list[Point3]) -> list[list[float]]:
    covariance = [[0.0, 0.0, 0.0] for _ in range(3)]
    for camera_point, robot_point in zip(camera_delta, robot_delta):
        for row in range(3):
            for col in range(3):
                covariance[row][col] += camera_point[row] * robot_point[col]
    sxx, sxy, sxz = covariance[0]
    syx, syy, syz = covariance[1]
    szx, szy, szz = covariance[2]
    trace = sxx + syy + szz
    symmetric = [
        [trace, syz - szy, szx - sxz, sxy - syx],
        [syz - szy, sxx - syy - szz, sxy + syx, szx + sxz],
        [szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy],
        [sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz],
    ]
    q = largest_eigenvector_4x4(symmetric)
    norm = math.sqrt(sum(value * value for value in q)) or 1.0
    w, x, y, z = [value / norm for value in q]
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def largest_eigenvector_4x4(matrix: list[list[float]]) -> list[float]:
    a = [row[:] for row in matrix]
    vectors = [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]
    for _ in range(50):
        pivot_row, pivot_col = 0, 1
        pivot = abs(a[pivot_row][pivot_col])
        for row in range(4):
            for col in range(row + 1, 4):
                value = abs(a[row][col])
                if value > pivot:
                    pivot = value
                    pivot_row, pivot_col = row, col
        if pivot < 1e-12:
            break
        app = a[pivot_row][pivot_row]
        aqq = a[pivot_col][pivot_col]
        apq = a[pivot_row][pivot_col]
        tau = (aqq - app) / (2.0 * apq)
        tangent = math.copysign(1.0 / (abs(tau) + math.sqrt(1.0 + tau * tau)), tau)
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine
        for index in range(4):
            if index not in {pivot_row, pivot_col}:
                aip = a[index][pivot_row]
                aiq = a[index][pivot_col]
                a[index][pivot_row] = cosine * aip - sine * aiq
                a[pivot_row][index] = a[index][pivot_row]
                a[index][pivot_col] = sine * aip + cosine * aiq
                a[pivot_col][index] = a[index][pivot_col]
        a[pivot_row][pivot_row] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq
        a[pivot_col][pivot_col] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq
        a[pivot_row][pivot_col] = 0.0
        a[pivot_col][pivot_row] = 0.0
        for index in range(4):
            vip = vectors[index][pivot_row]
            viq = vectors[index][pivot_col]
            vectors[index][pivot_row] = cosine * vip - sine * viq
            vectors[index][pivot_col] = sine * vip + cosine * viq
    eigen_index = max(range(4), key=lambda index: a[index][index])
    return [vectors[row][eigen_index] for row in range(4)]
