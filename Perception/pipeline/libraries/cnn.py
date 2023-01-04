import torch.nn as nn


# Small Keypoints CNN
class smallKeypointslargeKeypointsCNN_layer(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size_conv, kernel_size_pool, stride, padding, padding_pool):
        super(smallKeypointslargeKeypointsCNN_layer, self).__init__()

        self.conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=kernel_size_conv, stride=stride, padding=padding)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool2d(kernel_size=kernel_size_pool, padding=padding_pool)
    def forward(self, x):
        out = (self.pool(self.relu(self.conv(x))))
        return out

class smallKeypointsBasicCNN(nn.Module):
    def __init__(self, output_size = 14):
        super(smallKeypointsBasicCNN, self).__init__()

        self.conv1 = smallKeypointslargeKeypointsCNN_layer(in_channels=3, out_channels=32, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
        self.conv2 = smallKeypointslargeKeypointsCNN_layer(in_channels=32, out_channels=64, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
        self.conv3 = smallKeypointslargeKeypointsCNN_layer(in_channels=64, out_channels=128, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))

        self.linear = nn.Sequential(nn.Linear(in_features=3072, out_features=1024),
                                    nn.LeakyReLU(),
                                    nn.Dropout(0.1),
                                    nn.Linear(1024, 512),
                                    nn.LeakyReLU(),
                                    nn.Dropout(0.1),
                                    nn.Linear(512, 14))

    def forward(self, x):
        features = self.conv3(self.conv2(self.conv1(x)))
        features = features.view(features.shape[0], -1)
        return self.linear(features)



# Large Keypoints CNN
class largeKeypointsCNN_layer(nn.Module):
  def __init__(self, in_channels, out_channels, kernel_size_conv, kernel_size_pool, stride, padding, padding_pool):
    super(largeKeypointsCNN_layer, self).__init__()

    self.conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=kernel_size_conv, stride=stride, padding=padding)
    self.activ = nn.ReLU()
    self.pool = nn.MaxPool2d(kernel_size=kernel_size_pool, padding=padding_pool)

  def forward(self, x):
    out = (self.pool(self.activ(self.conv(x))))
    return out

class largeKeypointsBasicCNN( nn.Module):
  def __init__(self):
    super(largeKeypointsBasicCNN, self).__init__()

    self.conv1 = largeKeypointsCNN_layer(in_channels=3 , out_channels=32 , kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    self.conv2 = largeKeypointsCNN_layer(in_channels=32, out_channels=64 , kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    self.conv3 = largeKeypointsCNN_layer(in_channels=64, out_channels=128, kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    
    self.linear = nn.Sequential(nn.Linear(in_features = 1920, out_features = 1024),
                                nn.LeakyReLU(),
                                nn.Dropout(0.6),
                                nn.Linear(in_features = 1024, out_features = 512),
                                nn.LeakyReLU(),
                                nn.Dropout(0.6),
                                nn.Linear(in_features = 512, out_features = 22))
    
  def forward(self, x):
    features = self.conv3(self.conv2(self.conv1(x)))
    features = features.view(features.shape[0], -1)
    return self.linear(features)


# Kati alla
class VGG_layer(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size_conv1, kernel_size_conv2, stride1, stride2, padding):
        super(VGG_layer, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=kernel_size_conv1,
        stride=stride1, padding=padding)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=kernel_size_conv2,
        stride=stride2, padding=padding)
        self.relu2 = nn.ReLU()
        self.pool = nn.MaxPool2d((2, 2), 2)

    def forward(self, x):
        out = self.pool(self.relu2(self.conv2(self.relu1(self.conv1(x)))))
        return out

class VGGLikeV3(nn.Module):
    def __init__(self):
        super(VGGLikeV3, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=3, out_channels=64, kernel_size=(3, 3), stride=1, padding=0)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool2d((2, 2), 2)
        self.layer1 = VGG_layer(64, 128, 3, 3, 1, 1, 0)
        self.layer2 = VGG_layer(128, 256, 3, 3, 1, 1, 0)

        self.linear = nn.Sequential(nn.Linear(in_features=2048, out_features=1024),
        nn.ReLU(),
        nn.Dropout(0.3),
        nn.Linear(1024, 1024),
        nn.ReLU(),
        nn.Dropout(0.3),
        nn.Linear(1024, 14))

    def forward(self, x):
        features = self.layer2(self.layer1(self.pool1(self.relu1(self.conv1(x)))))
        features = features.view(features.shape[0], -1) # tensorflow equivalent to flatten()
        return self.linear(features)
