#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import os
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import gym
import torch
import torch.nn as nn
import torch.nn.functional as F
from gym import wrappers
from torch.autograd import Variable
from collections import deque


class ReplayBuffer(object):
    def __init__(self, max_size=1e6):
        self.storage = []
        self.max_size = max_size
        self.ptr = 0  # index of transitions (pointer)

    def add(self, transition):
        if len(self.storage) == self.max_size:
            self.storage[int(self.ptr)] = transition
            self.ptr = (self.ptr + 1) % self.max_size

        else:
            self.storage.append(transition)

    
        

    def sample(self, batch_size):
        ind = np.random.randint(0, len(self.storage), batch_size)

        batch_states, batch_next_states, batch_actions, batch_rewards, batch_done = [], [], [], [], []
        for i in ind:
            state, next_state, action, reward, done = self.storage[i]
            batch_states.append(np.array(state, copy=False))
            batch_next_states.append(np.array(next_state, copy=False))
            batch_actions.append(np.array(action, copy=False))
            batch_rewards.append(np.array(reward, copy=False))
            batch_done.append(np.array(done, copy=False))

        return np.array(batch_states), np.array(batch_next_states), np.array(batch_actions), np.array(
            batch_rewards).reshape(-1, 1), np.array(batch_done).reshape(-1, 1)


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 400)
        self.layer_2 = nn.Linear(400, 300)
        self.layer_3 = nn.Linear(300, action_dim)
        self.max_action = max_action

    def forward(self, x):
        x = F.relu(self.layer_1(x))
        x = F.relu(self.layer_2(x))



        # 3 3# 3 x = F.relu(self.layer_3(x))
        #print(x)
	
        #x = self.max_action * torch.tanh(x)
        x = self.layer_3(x)
        #print(x)
        x = self.max_action * torch.tanh(x)
        

        return x


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        self.layer_1 = nn.Linear(state_dim + action_dim, 400)
        self.layer_2 = nn.Linear(400, 300)
        self.layer_3 = nn.Linear(300, 1)

        self.layer_4 = nn.Linear(state_dim + action_dim, 400)
        self.layer_5 = nn.Linear(400, 300)
        self.layer_6 = nn.Linear(300, 1)

    def forward(self, x, u):
        
        #print(x.shape, u.shape)
        xu = torch.cat([x, u], 1)
        
        #xu = torch.flatten(xu, 1)

        x1 = F.relu(self.layer_1(xu))
        x1 = F.relu(self.layer_2(x1))
        x1 = self.layer_3(x1)

        x2 = F.relu(self.layer_4(xu))
        x2 = F.relu(self.layer_5(x2))
        x2 = self.layer_6(x2)
        return x1, x2

    ### backpropagation for critic networks
    def Q1(self, x, u):
        xu = torch.cat([x, u], 1)
        x1 = F.relu(self.layer_1(xu))
        x1 = F.relu(self.layer_2(x1))
        x1 = self.layer_3(x1)

        return x1


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class TD3(object):
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = Actor(state_dim, action_dim, max_action).to(device)
        self.actor_target = Actor(state_dim, action_dim, max_action).to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(),lr = 3e-4)

        self.critic = Critic(state_dim, action_dim).to(device)
        self.critic_target = Critic(state_dim, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(),lr = 3e-4)

        self.max_action = max_action

    def select_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()  ### same as self.actor.forward(state)

    def train(self, replay_buffer, iterations, batch_size=256, discount=0.99, tau=0.005, policy_noise=0.2, noise_clip=0.5, policy_freq=2):
        for i in range(iterations):

            batch_states, batch_next_states, batch_actions, batch_rewards, batch_dones = replay_buffer.sample(batch_size)
            state = torch.FloatTensor(batch_states).to(device)
            next_state = torch.FloatTensor(batch_next_states).to(device)
            action = torch.FloatTensor(batch_actions).to(device)
            reward = torch.FloatTensor(batch_rewards).to(device)
            done = torch.FloatTensor(batch_dones).to(device)

            next_action = self.actor_target(next_state)  #### same as self.actor_target.forward(next_state)

            noise = torch.FloatTensor(batch_actions).data.normal_(0, policy_noise).to(device)
            noise = noise.clamp(-noise_clip, noise_clip)
            next_action = (next_action + noise).clamp(-self.max_action, self.max_action)
	    print(next_action)

            qt1, qt2 = self.critic_target.forward(next_state, next_action)

            target_Q = torch.min(qt1, qt2)

            target_Q = reward + (discount * target_Q * (1 - done)).detach()

            q1, q2 = self.critic.forward(state, action)

            critic_loss = F.mse_loss(q1, target_Q) + F.mse_loss(q2, target_Q)

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

            if i % policy_freq == 0:
                actor_loss = -self.critic.Q1(state, self.actor.forward(state)).mean()
                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                self.actor_optimizer.step()

                for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                    target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

                for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                    target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

    def save(self, filename, directory):
        
        torch.save(self.actor.state_dict(), '%s/%s_actor' % (directory, filename))
        torch.save(self.critic_optimizer.state_dict(), '%s/%s_critic_optimizer' % (directory, filename))
        torch.save(self.actor_optimizer.state_dict(), '%s/%s_actor_optimizer' % (directory, filename))
 
        torch.save(self.critic.state_dict(), '%s/%s_critic' % (directory, filename))
       
        

        

    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load('%s/%s_actor' % (directory, filename)))
        self.actor_optimizer.load_state_dict(torch.load('%s/%s_actor_optimizer' % (directory, filename)))
        self.actor_target = copy.deepcopy(self.actor)

        self.critic.load_state_dict(torch.load('%s/%s_critic' % (directory, filename)))
        self.critic_optimizer.load_state_dict(torch.load('%s/%s_critic_optimizer' % (directory, filename)))
        self.critic_target = copy.deepcopy(self.critic)
        
        
