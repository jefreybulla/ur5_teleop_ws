"""Print UR5 end-effector pose from forward kinematics."""

import math

from ur5_kinematics.forward_kinematics import ur5_dirkin


def deg2rad(degrees):
    """Convert joint angles from degrees to radians."""
    if isinstance(degrees, (list, tuple)):
        return [deg2rad(d) for d in degrees]
    return math.radians(degrees)


def main(args=None):
    del args  # ROS2 console_scripts pass unused argv

    q_deg = [20,-100,110,0,0,0]
    q = deg2rad(q_deg)

    x, y, z, axang, frames = ur5_dirkin(q)

    print('Joint angles q [deg] -> [rad]:')
    for qd, qr in zip(q_deg, q):
        print(f'  {qd:+8.2f} deg  ->  {qr:+.4f} rad')
    print()
    print('End-effector position [m]:')
    print(f'  x = {x:.6f}')
    print(f'  y = {y:.6f}')
    print(f'  z = {z:.6f}')
    print()
    ux, uy, uz, theta = axang
    print('Orientation (axis-angle):')
    print(f'  axis  = [{ux:.6f}, {uy:.6f}, {uz:.6f}]')
    print(f'  angle = {theta:.6f} rad ({math.degrees(theta):.2f} deg)')
    print()
    t_ee = frames[:, :, 5]
    print('End-effector homogeneous transform ^0T_6:')
    print('  [ R | t ]')
    for row in range(4):
        print(
            '  [ '
            + '  '.join(f'{t_ee[row, col]:+10.6f}' for col in range(4))
            + ' ]'
        )
    print()
    print('Link frames ^0T_i (translation only):')
    for i in range(frames.shape[2]):
        tx, ty, tz = frames[:3, 3, i]
        print(f'  link {i + 1}: [{tx:.6f}, {ty:.6f}, {tz:.6f}]')


if __name__ == '__main__':
    main()
