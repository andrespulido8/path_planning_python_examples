import numpy as np

def potential_field(p0, pf, obstacles):
    """ Creates a potential field by summing the repulsive and attractive fields.
        The attractive field is a quadratic function of the distance to the goal.
        The repulsive field is a function of the distance to the obstacles.
    """
    p0 = np.array(p0)
    pf = np.array(pf)
    # attractive force 
    psi = 1  # scaling factor
    f_att = lambda p: - psi*(pf - p)/np.linalg.norm(pf - p)

    # include the edges of the obstacles
    obs_points = np.array([]).reshape(0, p0.shape[0] + 1)
    for jj, obstacle in enumerate(obstacles):
        # add final point to close the polygon
        obstacle = np.vstack([obstacle, obstacle[0]])
        for ii in range(len(obstacle) - 1):
            segment = np.linspace(obstacle[ii], obstacle[(ii + 1)], 10)[:-1]
            # add the obstacle id to the points
            segment = np.hstack([segment, np.ones((len(segment), 1)) * jj])
            obs_points = np.vstack([obs_points, segment])

    def repulsive_force(p):
        eta = 1  # scaling factor

        # argmin of the distance to the obstacles
        idx = np.argmin(np.linalg.norm(p - obs_points[:, :2], axis=1))
        closest_point = obs_points[idx, :2]
        d = np.linalg.norm(p - closest_point)

        threshold = 0.5
        if d > threshold:
            f = np.zeros(2)
        else:
            f =  - eta * (1/d - 1/threshold) * (1/d**2) * (p - closest_point)
            #print(f"p: {p}, closest_point: {closest_point}, d: {d}")

        return f

    def total_f(p):
        # potential field force
        f_rep = repulsive_force(np.array(p))
        f_attr = f_att(p)
        #print(f"repulsive force: {f_rep} and attractive force: {f_attr}")
        return f_attr, f_rep

    # perform gradient descent to find the path
    alpha = 0.01  # step size
    path = [p0]
    p = p0
    count = 0
    while np.linalg.norm(p - pf) > 0.1:
        #if there is no progress, move perpendicular to the force
        if len(path) > 5:
            prev_path_array = np.array(path[-5:-1])
            # if the last 5 points are close to each other, we are stuck
            displacement = 0
            for i in range(3):
                displacement += np.linalg.norm(prev_path_array[i] - prev_path_array[i + 1])
            if displacement < 0.1/10 and count < 10: 
                print(f"No progress, displacement in the last 5 steps is {displacement}.\nMoving perpendicular to the repulsive force")
                fa, fr = total_f(p)
                fr = np.array([-fr[1], fr[0]])
                fr = fr / np.linalg.norm(fr) if np.linalg.norm(fr) > 0 else fr
                p = p + (fa+0.5*fr) * 0.1
                count += 1
            else:
                p = p - alpha * np.sum(total_f(p), axis=0)
            if count == 10:
                print("Going too far from the path")
                count = 0
        else:
            p = p - alpha * np.sum(total_f(p), axis=0)
        path.append(p)
        #print(f"p: {p}")
        #print("len(path): ", len(path))
        if len(path) > 2000:
            print("No path found")
            return total_f, path

    print(f"Path found using potential field! in {len(path)} steps.")
    return total_f, path
