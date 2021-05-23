require 'image'
if not dqn then
    require "initenv"
end
require 'torch'
function initialize()
  agent_params = {
    -- The agent only knows about actions it can take in the environment
    -- we will use gpu
    gpu = -1,
    -- no. of valid actions
    n_actions = 4,
    -- we will print info
    verbose = 2,
    -- learning rate for SGD
    lr=0.00025,
    -- Random exploration ratio, start from 100% exploration
    ep=1,
    -- Drop down to 10% exploration
    ep_end=0.1,
    -- Linear decay over 1M steps - in our case 20,000 steps
    ep_endt=20000,
    -- Discount factor \gamma for Q-Learning
    discount=0.2,
    -- Number of frames to input into convolutional net, i.e. history lenth 
    hist_len=1,
    -- Learning starts after a delay of 50K actions, we do not want to overfit onto early experience, in our case 1000 actions
    learn_start=1000,
    -- We will store last 1M transitions
    replay_memory=50000,
    -- We will update every 4 actions, in our case every action
    update_freq=1,
    -- Will update only once
    n_replay=1,
    -- Network spec
    --network= "/home/somdyuti/gazebo_test/Networks3/DQN_24000_agent.t7",
    network = "convnet_atari3",
    --pre-processing spec (just scale down to grayscale 84x84)

    preproc="net_downsample_2x_full_y",
    -- size of inputs after rescale (84*84)
    state_dim=7056,
    -- size of minibatch for SGD
    minibatch_size=100,
    -- we will scale reward values to limit to 1,-1. NO for now
    rescale_r=0,
    -- we use Y channel
    ncols=1,
    -- buffer on GPU
    bufferSize=512,

   -- set of validation transitions to track training progress
    valid_size=500,
    -- update target Q network every 10K updates, 500 updates in our case
    target_q=500,
    -- we will clip errors that go into DQN
    clip_delta=1,
    -- clip reward between -1,1
    min_reward=-1,
    -- clip reward between -1,1
    max_reward=1
}
agent = dqn.DDPGLearner(agent_params)
print("agent ", agent)
agent:setup()
end


function get_action_index(reward, terminal)
 print("Reward ", reward)
 --print("terminal", terminal)
 if terminal==0 then
 terminal = false
 else
 terminal = true
 end

 --print("reward " , reward)
 --print("terminal ", terminal)
 img = image.load("/home/sudhir/DDPG_Networks/images/Concatenated_image.png", 3, 'double')
 screen = img
 --image.display(screen)
 --print(screen)
 local action_index, err = agent:perceive(reward, screen, terminal, 'true')

 action_index = 50*action_index

 if not terminal then
 return err, action_index[4],action_index[3],action_index[2],action_index[1]
 else
 --[[local act_ind = torch.FloatTensor(4)
 act_ind[1] = 50*torch.randn(1)
 act_ind[2] = 50*torch.randn(1)
 act_ind[3] = 50*torch.randn(1)
 act_index[4] = 50*torch.randn(1)]]--
 return err, 0, 0, 0, 0
 end
end

