import matplotlib.pyplot as plt


def visualize_hinge_and_center_of_mass(
    hinge_upper_point, hinge_lower_point, origin, center_of_mass
):
    fig = plt.figure()
    ax = plt.axes(projection="3d")

    ax.scatter3D(
        center_of_mass[0],
        center_of_mass[1],
        center_of_mass[2],
        "green",
        label="Center of Mass",
    )
    ax.plot3D(
        [hinge_upper_point[0] - hinge_lower_point[0], 0],
        [hinge_upper_point[1] - hinge_lower_point[1], 0],
        [hinge_upper_point[2] - hinge_lower_point[2], 0],
        "brown",
        label="Rotation Axis",
    )
    ax.scatter3D(
        origin[0],
        origin[1],
        origin[2],
        "orange",
        label="origin",
    )
    ax.plot3D(
        [origin[0], center_of_mass[0]],
        [origin[1], center_of_mass[1]],
        [origin[2], center_of_mass[2]],
        "orange",
        label="cm to origin",
    )

    plt.legend()
    plt.show()
