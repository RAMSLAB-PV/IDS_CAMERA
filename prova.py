import numpy as np
import cv2
import pyceres


# ref: examples/helloworld_analytic_diff.cc
class HelloworldCostFunction(pyceres.CostFunction):
    def __init__(self):
        pyceres.CostFunction.__init__(self)
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1,1])

    def Evaluate(self, parameters, residuals, jacobians):
        x = parameters[0][0]
        res1= 10.0 - x
        y = parameters[1][0]
        res2= 25.0 - y
        residuals[0] = res1*res2
        if jacobians is not None:
            jacobians[0][0] = 0
            jacobians[1][0] = -1*res1
        return True


def test_python_cost():
    x = np.array(5.0)
    x_ori = x.copy()
    y = np.array(10.0)
    y_ori = y.copy()
    prob = pyceres.Problem()
    cost = HelloworldCostFunction()
    prob.add_residual_block(cost, None, [x,y])
    options = pyceres.SolverOptions()
    options.minimizer_progress_to_stdout = True
    summary = pyceres.SolverSummary()
    pyceres.solve(options, prob, summary)
    print(summary.BriefReport())
    print(f"{x_ori} -> {x}")
    print(f"{y_ori} -> {y}")

class ReprojectionCostFunction(pyceres.CostFunction):
    def __init__(self):
        super().__init__()
        self.set_num_residuals(1)
        self.set_parameter_block_sizes([1, 1, 1, 1, 1, 1])


    def Evaluate(self, parameters, residuals, jacobians):
        observed = np.array([[1027.9907 ,  794.9041 ],
            [1027.2947 ,  703.91406],
            [1026.497  ,  612.5368 ],
            [1118.2146 ,  796.8792 ],
            [1117.9833 ,  704.3839 ],
            [1116.9982 ,  611.0883 ],
            [1210.026  ,  798.0821 ],
            [1210.4116 ,  704.53754],
            [1209.9893 ,  610.0088 ],
            [1303.1099 ,  798.3448 ],
            [1304.1499 ,  704.11066],
            [1303.1763 ,  608.7239 ],
            [1396.6923 ,  798.1369 ],
            [1397.5958 ,  703.3377 ],
            [1396.9244 ,  607.9472 ],
            [1489.0524 ,  797.2684 ],
            [1490.789  ,  702.0262 ],
            [1490.0833 ,  606.24005]])
        
        world_point = np.array([[  0. ,   0. ,   0. ],
            [ 50.8,   0. ,   0. ],
            [101.6,   0. ,   0. ],
            [  0. ,  50.8,   0. ],
            [ 50.8,  50.8,   0. ],
            [101.6,  50.8,   0. ],
            [  0. , 101.6,   0. ],
            [ 50.8, 101.6,   0. ],
            [101.6, 101.6,   0. ],
            [  0. , 152.4,   0. ],
            [ 50.8, 152.4,   0. ],
            [101.6, 152.4,   0. ],
            [  0. , 203.2,   0. ],
            [ 50.8, 203.2,   0. ],
            [101.6, 203.2,   0. ],
            [  0. , 254. ,   0. ],
            [ 50.8, 254. ,   0. ],
            [101.6, 254. ,   0. ]])
        
        fx, fy, cx, cy, k1, k2 = parameters

        X, Y, Z = world_point[:, 0], world_point[:, 1], world_point[:, 2]

        x = fx * (X / Z) + cx
        y = fy * (Y / Z) + cy
        
        # Compute distortion
        r2 = x**2 + y**2
        x_distorted = x * (1 + k1 * r2 + k2 * r2**2)
        y_distorted = y * (1 + k1 * r2 + k2 * r2**2)
        
        # Compute residuals
        ox, oy = observed[:, 0], observed[:, 1]
        residuals[0] = np.mean((ox - x_distorted)**2 + (oy - y_distorted)**2)
        

        if jacobians is not None:

            # Compute Jacobians w.r.t. intrinsic parameters
            jacobian_intrinsic = np.zeros((18, 6))
            jacobian_intrinsic[:, 0] = -2 * (ox - x_distorted) * (X / Z) * (1 + k1 * r2 + k2 * r2**2)  # df/dfx
            jacobian_intrinsic[:, 1] = -2 * (oy - y_distorted) * (Y / Z) * (1 + k1 * r2 + k2 * r2**2)  # df/dfy
            jacobian_intrinsic[:, 2] = -2 * (ox - x_distorted) * (1 + k1 * r2 + k2 * r2**2)  # df/dcx
            jacobian_intrinsic[:, 3] = -2 * (oy - y_distorted) * (1 + k1 * r2 + k2 * r2**2)  # df/dcy
            jacobian_intrinsic[:, 4] = -2 * (ox - x_distorted) * (r2)  # df/dk1
            jacobian_intrinsic[:, 5] = -2 * (oy - y_distorted) * (r2**2)  # df/dk2


            jacobians[0][0] = np.mean(jacobian_intrinsic[:, 0]) 
            jacobians[1][0] = np.mean(jacobian_intrinsic[:, 1]) 
            jacobians[2][0] = np.mean(jacobian_intrinsic[:, 2]) 
            jacobians[3][0] = np.mean(jacobian_intrinsic[:, 3])
            jacobians[4][0] = np.mean(jacobian_intrinsic[:, 4])
            jacobians[5][0] = np.mean(jacobian_intrinsic[:, 5])

        return True



if __name__ == "__main__":
    test_python_cost()
    initial_params = {'fx': 1000.0,
                    'fy': 1000.0,
                    'cx': 320.0,
                    'cy': 240.0,
                    'k1': 0.1,
                    'k2': 0.01,
                    'rvec': np.array([1.0, 1.0, 1.0]),
                    'tvec': np.array([1.0, 1.0, 1.0]),
                    }
    
    params = [
    np.array([initial_params['fx']]),
    np.array([initial_params['fy']]),
    np.array([initial_params['cx']]),
    np.array([initial_params['cy']]),
    np.array([initial_params['k1']]),
    np.array([initial_params['k2']])]

    calibration = pyceres.Problem()

    cost = ReprojectionCostFunction()
    calibration.add_residual_block(cost, None, params)

    options = pyceres.SolverOptions()
    options.minimizer_progress_to_stdout = True
    summary = pyceres.SolverSummary()
    pyceres.solve(options, calibration, summary)
    print(summary.BriefReport())

