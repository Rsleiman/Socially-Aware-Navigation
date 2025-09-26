#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
import numpy as np

#TODO: Adapted to work with full observation space (4 * 240 laser + 2 position)


class ConvLSTMA2CNetwork(nn.Module):
    """
    Advanced Actor-Critic Network with Conv1D + LSTM for Social Navigation
    
    Architecture:
    - Input: 962-dimensional observation (4*240 fused array + 2 position) 
    - Conv1D layers process each observation type (agents, obstacles, groups, goals) separately
    - LSTM processes conv outputs combined with position data
    - FC layers output actor and critic values
    """
    
    def __init__(self, obs_dim=962, fused_array_dim=960, action_dim=3, lstm_hidden=64):
        super(ConvLSTMA2CNetwork, self).__init__()
        
        self.obs_dim = obs_dim
        self.fused_array_dim = fused_array_dim
        self.position_dim = obs_dim - fused_array_dim  # Should be 2 (x, y)
        self.action_dim = action_dim
        self.lstm_hidden = lstm_hidden
        
        self.rays_per_type = 240
        self.num_observation_types = 4  # agents, obstacles, group_ids, goals
        
        self.conv_layers = nn.Sequential(
            nn.Conv1d(in_channels=self.num_observation_types, out_channels=32, kernel_size=7, stride=2),
            nn.ReLU(),
            nn.Conv1d(in_channels=32, out_channels=64, kernel_size=5, stride=2), 
            nn.ReLU(),
            nn.Conv1d(in_channels=64, out_channels=64, kernel_size=3, stride=2),
            nn.ReLU()
        )
        
        # Calculate conv output size
        conv_output_size = self.get_conv_output_size()
        
        # LSTM layer (processes conv output + position data)
        lstm_input_size = conv_output_size + self.position_dim
        self.lstm = nn.LSTM(input_size=lstm_input_size, hidden_size=lstm_hidden, batch_first=True)
        
        # Fully connected layers
        self.fc1 = nn.Linear(lstm_hidden, 512)
        self.fc2 = nn.Linear(512, 512)
        
        # Actor head (policy)
        self.actor_mean = nn.Linear(512, action_dim)
        self.actor_log_std = nn.Parameter(torch.zeros(action_dim))
        
        # Critic head (value function)
        self.critic = nn.Linear(512, 1)
        
        # LSTM hidden states 
        self.hidden_state = None
        self.cell_state = None
        
        self.initialise_weights()

    def get_conv_output_size(self): # In case we change architecture
        # Simulate forward pass through conv layers with 4 observation types
        with torch.no_grad():
            dummy_input = torch.randn(1, self.num_observation_types, self.rays_per_type)
            conv_output = self.conv_layers(dummy_input)
            return conv_output.view(conv_output.size(0), -1).size(1)

    def initialise_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=0.5)
                nn.init.constant_(m.bias, 0.0)
            elif isinstance(m, nn.Conv1d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0.0)
            elif isinstance(m, nn.LSTM):
                for name, param in m.named_parameters():
                    if 'weight' in name:
                        nn.init.orthogonal_(param)
                    elif 'bias' in name:
                        nn.init.constant_(param, 0.0)
        
        nn.init.orthogonal_(self.actor_mean.weight, gain=0.01)
        nn.init.constant_(self.actor_mean.bias, 0.0)
    
    def reset_hidden_states(self, batch_size=1):
        device = next(self.parameters()).device
        self.hidden_state = torch.zeros(1, batch_size, self.lstm_hidden, device=device)
        self.cell_state = torch.zeros(1, batch_size, self.lstm_hidden, device=device)
        
    def forward(self, observations, reset_hidden=False):
        batch_size = observations.size(0)
        
        # Reset  if requested or if first time
        if reset_hidden or self.hidden_state is None:
            self.reset_hidden_states(batch_size)
        
        # Split observation into fused array data and position
        fused_array_data = observations[:, :self.fused_array_dim]  # Shape: [batch, 960]
        position_data = observations[:, self.fused_array_dim:]  # Shape: [batch, 2]
        
        # Reshape fused array into 4 channels for conv processing
        fused_reshaped = fused_array_data.view(batch_size, self.num_observation_types, self.rays_per_type)  # Shape: [batch, 4, 240]
        
        # Process layers
        conv_output = self.conv_layers(fused_reshaped)  # Shape: [batch, 64, conv_seq_len]
        
        # Flatten output
        conv_flattened = conv_output.view(batch_size, -1)  # Shape: [batch, conv_output_size]
        
        # Combine conv output with position data
        lstm_input = torch.cat([conv_flattened, position_data], dim=1)  # Shape: [batch, lstm_input_size]
        lstm_input = lstm_input.unsqueeze(1)  # Shape: [batch, 1, lstm_input_size] for sequence
        
        # LSTM forward pass
        lstm_output, (self.hidden_state, self.cell_state) = self.lstm(
            lstm_input, (self.hidden_state, self.cell_state)
        )
        lstm_output = lstm_output.squeeze(1)  # Shape: [batch, lstm_hidden]
        
        # FCLs
        features = F.relu(self.fc1(lstm_output))
        features = F.relu(self.fc2(features))
        
        # Actor output (actions)
        action_mean = self.actor_mean(features)
        action_log_std = self.actor_log_std.expand_as(action_mean)
        action_std = torch.exp(action_log_std)
        
        # Critic output (state value)
        state_value = self.critic(features)
        
        return action_mean, action_std, state_value
    
    def get_action_and_value(self, observations, action=None, reset_hidden=False):       
        action_mean, action_std, state_value = self.forward(observations, reset_hidden)
        
        # Create action distribution
        action_dist = Normal(action_mean, action_std)
        
        if action is None:
            # Sample action for exploration
            action = action_dist.sample()
        
        # Compute log probability of the action
        action_log_prob = action_dist.log_prob(action).sum(dim=-1)
        
        return action, action_log_prob, action_dist.entropy().sum(dim=-1), state_value.squeeze(-1)
    
    def get_value(self, observations, reset_hidden=False):
        _, _, state_value = self.forward(observations, reset_hidden)
        return state_value.squeeze(-1)


class A2CAgent:    
    def __init__(self, obs_dim=962, action_dim=3, learning_rate=3e-4, device='cpu'):
        self.device = torch.device(device)
        
        self.network = ConvLSTMA2CNetwork(obs_dim, action_dim=action_dim).to(self.device)
        
        # Optimiser
        self.optimiser = torch.optim.Adam(self.network.parameters(), lr=learning_rate)
        
        # Hyperparameters
        self.gamma = 0.99  # Discount factor
        self.gae_lambda = 0.95  # GAE parameter
        self.clip_grad_norm = 0.5  # Gradient clipping
        self.value_loss_coef = 0.5  # Value loss coefficient
        self.entropy_coef = 0.01  # Entropy bonus coefficient
        
    def predict(self, observation, deterministic=False, action_space=None, reset_hidden=False):
        """Predict action given observation"""
        self.network.eval()
        
        with torch.no_grad():
            obs_tensor = torch.FloatTensor(observation).unsqueeze(0).to(self.device)
            
            if deterministic:
                action_mean, _, _ = self.network.forward(obs_tensor, reset_hidden)
                action = torch.tanh(action_mean)  # Bound actions to [-1, 1]
            else:
                action, _, _, _ = self.network.get_action_and_value(obs_tensor, reset_hidden=reset_hidden)
                action = torch.tanh(action)  # Bound actions to [-1, 1]
        
        # Scale actions from [-1, 1] to actual action space bounds
        if action_space is not None:
            action_np = action.cpu().numpy().flatten()
            # Scale from [-1, 1] to [action_space.low, action_space.high]
            scaled_action = action_space.low + (action_np + 1.0) * 0.5 * (action_space.high - action_space.low)
            return scaled_action
        else:
            return action.cpu().numpy().flatten()

    # Generalised Advantage Estimation (GAE)    
    def compute_gae(self, rewards, values, dones, next_value):
        advantages = torch.zeros_like(rewards)
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value_t = next_value
            else:
                next_value_t = values[t + 1]
                
            delta = rewards[t] + self.gamma * next_value_t * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages[t] = gae
            
        returns = advantages + values
        return advantages, returns
    
    # Update network with experience batch 
    def update(self, observations, actions, rewards, dones, values, action_space=None):       
        self.network.train() # sets to training mode
        
        # Convert to tensors
        obs_tensor = torch.FloatTensor(observations).to(self.device)
        
        # Scale actions back to [-1, 1]
        if action_space is not None:
            # [action_space.low, action_space.high] to [-1, 1]
            scaled_actions = 2.0 * (actions - action_space.low) / (action_space.high - action_space.low) - 1.0
            actions_tensor = torch.FloatTensor(scaled_actions).to(self.device)
        else:
            actions_tensor = torch.FloatTensor(actions).to(self.device)
            
        rewards_tensor = torch.FloatTensor(rewards).to(self.device)
        dones_tensor = torch.FloatTensor(dones).to(self.device)
        old_values_tensor = torch.FloatTensor(values).to(self.device)
        
        #  next value needed for GAE calculation
        with torch.no_grad():
            next_value = self.network.get_value(obs_tensor[-1:], reset_hidden=False)

        # Compute advantages and returns
        advantages, returns = self.compute_gae(rewards_tensor, old_values_tensor, dones_tensor, next_value)
        
        # Normalise
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Reset hidden states and process each step in the batch sequentially
        all_log_probs = []
        all_entropy = []
        all_values = []
        
        self.network.reset_hidden_states(1)
        
        for i in range(obs_tensor.size(0)):
            _, log_prob, entropy, value = self.network.get_action_and_value(
                obs_tensor[i:i+1], actions_tensor[i:i+1], reset_hidden=False
            )
            all_log_probs.append(log_prob)
            all_entropy.append(entropy)
            all_values.append(value)
        
        # Concatenate results
        log_probs = torch.cat(all_log_probs, dim=0)
        entropy = torch.cat(all_entropy, dim=0)
        values = torch.cat(all_values, dim=0)
        
        # Calculate losses
        value_loss = F.mse_loss(values, returns)
        policy_loss = -(log_probs * advantages).mean()
        entropy_loss = -entropy.mean()
        
        total_loss = policy_loss + self.value_loss_coef * value_loss + self.entropy_coef * entropy_loss
        
        # Backward pass
        self.optimiser.zero_grad()
        total_loss.backward()
        
        # Gradient clipping
        torch.nn.utils.clip_grad_norm_(self.network.parameters(), self.clip_grad_norm)
        
        self.optimiser.step()
        
        return {
            'total_loss': total_loss.item(),
            'policy_loss': policy_loss.item(),
            'value_loss': value_loss.item(),
            'entropy_loss': entropy_loss.item(),
            'mean_advantage': advantages.mean().item(),
            'mean_return': returns.mean().item()
        }
    
    def reset_lstm_states(self):
        self.network.reset_hidden_states(1)
    
    def save(self, path):
        torch.save({
            'network_state_dict': self.network.state_dict(),
            'optimiser_state_dict': self.optimiser.state_dict(),
        }, path)
        
    
    # For starting from a pre-trained model
    def load(self, path):
        checkpoint = torch.load(path, map_location=self.device)
        self.network.load_state_dict(checkpoint['network_state_dict'])
        self.optimiser.load_state_dict(checkpoint['optimiser_state_dict'])
