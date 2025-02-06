import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GCNConv, EdgeConv
from torch_geometric.data import Data, DataLoader
import numpy as np
from pathlib import Path
from ultralytics import YOLO
import cv2
from typing import List, Tuple, Dict
import json

class KeyboardGNN(nn.Module):
    def __init__(self, input_dim=5, hidden_dim=64, output_dim=2):
        super(KeyboardGNN, self).__init__()
        
        # Node feature processing
        self.node_encoder = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.LayerNorm(hidden_dim)
        )
        
        # Graph convolution layers
        self.conv1 = GCNConv(hidden_dim, hidden_dim)
        self.conv2 = GCNConv(hidden_dim, hidden_dim)
        self.conv3 = GCNConv(hidden_dim, hidden_dim)
        
        # Edge convolution for spatial relationships
        self.edge_conv = EdgeConv(nn.Sequential(
            nn.Linear(2 * hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim)
        ))
        
        # Output layers
        self.position_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)  # x, y position corrections
        )
        
        self.confidence_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
            nn.Sigmoid()
        )

class KeyboardDataset:
    def __init__(self, data_path: Path, yolo_model: YOLO):
        self.data_path = Path(data_path)
        self.yolo_model = yolo_model
        self.class_names = yolo_model.names
        
    def create_graph(self, detections: List, ground_truth: Dict) -> Data:
        """Convert YOLO detections to graph data"""
        num_keys = len(detections)
        
        # Node features: [x, y, w, h, class_onehot]
        node_features = []
        edge_index = []
        
        # Create node features
        for det in detections:
            x1, y1, x2, y2 = det.xyxy[0]
            w, h = x2 - x1, y2 - y1
            center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
            
            # One-hot encode class
            class_id = int(det.cls[0])
            class_onehot = F.one_hot(torch.tensor(class_id), 
                                   num_classes=len(self.class_names))
            
            features = torch.tensor([center_x, center_y, w, h] + 
                                  class_onehot.tolist())
            node_features.append(features)
        
        # Create edges between nearby keys
        for i in range(num_keys):
            for j in range(i + 1, num_keys):
                # Connect if keys are close enough
                dist = torch.norm(node_features[i][:2] - node_features[j][:2])
                if dist < 50:  # Threshold based on keyboard layout
                    edge_index.append([i, j])
                    edge_index.append([j, i])  # Add both directions
        
        # Create target positions from ground truth
        target_pos = []
        for det in detections:
            key_id = int(det.cls[0])
            key_name = self.class_names[key_id]
            if key_name in ground_truth:
                gt_x, gt_y = ground_truth[key_name]
                target_pos.append([gt_x, gt_y])
            else:
                # If no ground truth, use detected position
                x1, y1, x2, y2 = det.xyxy[0]
                target_pos.append([(x1 + x2) / 2, (y1 + y2) / 2])
        
        return Data(
            x=torch.stack(node_features),
            edge_index=torch.tensor(edge_index).t().contiguous(),
            pos=torch.tensor(target_pos)
        )
    
    def __getitem__(self, idx):
        img_path = list(self.data_path.glob('*.jpg'))[idx]
        label_path = img_path.with_suffix('.json')
        
        # Load image and run YOLO
        img = cv2.imread(str(img_path))
        results = self.yolo_model(img)[0]
        
        # Load ground truth
        with open(label_path) as f:
            ground_truth = json.load(f)
        
        # Create graph data
        return self.create_graph(results.boxes, ground_truth)

def train_gnn(model: KeyboardGNN, train_loader: DataLoader, 
              val_loader: DataLoader, num_epochs: int = 100):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = model.to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    
    # Loss functions
    mse_loss = nn.MSELoss()
    
    for epoch in range(num_epochs):
        model.train()
        total_loss = 0
        
        for batch in train_loader:
            batch = batch.to(device)
            optimizer.zero_grad()
            
            # Forward pass
            node_features = model.node_encoder(batch.x)
            conv1 = F.relu(model.conv1(node_features, batch.edge_index))
            conv2 = F.relu(model.conv2(conv1, batch.edge_index))
            conv3 = F.relu(model.conv3(conv2, batch.edge_index))
            
            # Edge features
            edge_features = model.edge_conv(conv3, batch.edge_index)
            
            # Predictions
            pos_correction = model.position_head(conv3)
            confidence = model.confidence_head(conv3)
            
            # Losses
            position_loss = mse_loss(batch.pos + pos_correction, batch.pos)
            
            # Spatial consistency loss
            spatial_loss = 0
            for i in range(0, batch.edge_index.size(1), 2):
                idx1, idx2 = batch.edge_index[:, i]
                pred_dist = torch.norm(pos_correction[idx1] - pos_correction[idx2])
                true_dist = torch.norm(batch.pos[idx1] - batch.pos[idx2])
                spatial_loss += mse_loss(pred_dist, true_dist)
            
            # Total loss
            loss = position_loss + 0.1 * spatial_loss
            
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
        
        # Validation
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for batch in val_loader:
                batch = batch.to(device)
                # Similar forward pass and loss computation
                # ...
        
        print(f'Epoch {epoch}: Train Loss = {total_loss/len(train_loader):.4f}, '
              f'Val Loss = {val_loss/len(val_loader):.4f}')

def main():
    # Initialize YOLO model
    yolo_model = YOLO('trained_yolov8n.pt')
    
    # Create datasets
    train_dataset = KeyboardDataset(Path('path/to/train'), yolo_model)
    val_dataset = KeyboardDataset(Path('path/to/val'), yolo_model)
    
    # Create dataloaders
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32)
    
    # Initialize and train model
    model = KeyboardGNN(
        input_dim=len(yolo_model.names) + 4,  # 4 spatial features + class one-hot
        hidden_dim=64,
        output_dim=2  # x, y position corrections
    )
    
    train_gnn(model, train_loader, val_loader)

if __name__ == '__main__':
    main()