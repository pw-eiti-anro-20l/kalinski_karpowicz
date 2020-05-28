        a, d, alpha, theta = dh[row]

        x_transl = translation_matrix((a, 0, 0))
        x_rot = rotation_matrix(alpha, X)
        z_transl = translation_matrix((0, 0, d))
        z_rot = rotation_matrix(theta, Z)

        transformation = concatenate_matrices(x_transl, x_rot, z_transl, z_rot)

        rpy = euler_from_matrix(transformation)
        xyz = translation_from_matrix(transformation)