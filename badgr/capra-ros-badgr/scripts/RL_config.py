
def get_params():
    control_freq = 5 # Hz

    batches = 500
    horizon = 2*control_freq
    init_vel = 1.5 # m/s
    init_steer = 0.0 # rad
    gamma = 20 # action update weight
    action_variance = (0.3, 1.5) # sampling variance (speed, steering)

    goal_gain = 0.25 # magic number - set to 0 for no target location reward
    action_gain = 0.2 # magic number - set to 0 for no action cost
    
    wheelbase = 0.16 # TODO Check with some one about this one ************

    model_name = 'carla23-04-2022--14:57--from09:34.pth'

    d = {
    'Control Freq': control_freq, 'Batches': batches, 'Horizon': horizon, 'Initial Speed': init_vel, 
    'Initial Steer Angle': init_steer, 'Gamma': gamma,'Action Sample Var': action_variance ,'Goal Cost Gain': goal_gain, 
    'Action Cost Gain': action_gain,'Wheel Base': wheelbase, 'Model Name': model_name,
    }

    return d

