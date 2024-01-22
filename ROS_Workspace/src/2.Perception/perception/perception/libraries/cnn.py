import torch.nn as nn

class CNN_layer(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size_conv, kernel_size_pool, stride, padding, padding_pool):
        super(CNN_layer, self).__init__()

        self.conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=kernel_size_conv, stride=stride, padding=padding)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool2d(kernel_size=kernel_size_pool, padding=padding_pool)
    def forward(self, x):
        out = (self.pool(self.relu(self.conv(x))))
        return out

class SmallCNN(nn.Module):
    def __init__(self, output_size = 14):
        super(SmallCNN, self).__init__()

        self.conv1 = CNN_layer(in_channels=3, out_channels=32, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
        self.conv2 = CNN_layer(in_channels=32, out_channels=64, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
        self.conv3 = CNN_layer(in_channels=64, out_channels=128, kernel_size_conv=(3,3), kernel_size_pool=(2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))

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


class LargeCNN( nn.Module):
  def __init__(self):
    super(LargeCNN, self).__init__()

    self.conv1 = CNN_layer(in_channels=3 , out_channels=32 , kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    self.conv2 = CNN_layer(in_channels=32, out_channels=64 , kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    self.conv3 = CNN_layer(in_channels=64, out_channels=128, kernel_size_conv=(4,4), kernel_size_pool= (2,2), stride=(1,1), padding=(0,0), padding_pool=(0,0))
    
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


# Neural Parh
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

class KeypointNet(nn.Module):
    def __init__(self):
        super(KeypointNet, self).__init__()
        net_size = 16
        #For ResNetMSEAugmSize8
        #self.conv = nn.Conv2d(in_channels=3, out_channels=net_size, kernel_size=5, stride=1, padding='same')
        #
        self.conv = nn.Conv2d(in_channels=3, out_channels=net_size, kernel_size=7, stride=1, padding='same')
        self.relu = nn.ReLU()
        self.res1 = ResNet(net_size, net_size)
        self.res2 = ResNet(net_size, net_size * 2)
        self.res3 = ResNet(net_size * 2, net_size * 4)
        self.res4 = ResNet(net_size * 4, net_size * 8)
        self.out = nn.Conv2d(in_channels=net_size * 8, out_channels=7, kernel_size=1, stride=1, padding=0)
        self.softmax = nn.Softmax(dim=1)

    def forward(self, x):
        act1 = self.relu(self.conv(x))
        act2 = self.res1(act1)
        act3 = self.res2(act2)
        act4 = self.res3(act3)
        act5 = self.res4(act4)
        hm = self.softmax(self.out(act5))
        return hm.view(-1, 7, 64, 48)

class ResNet(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(ResNet, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=2,
                               dilation=2)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1)
        self.relu2 = nn.ReLU()

        self.shortcut_conv = nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=1, stride=1)

    def forward(self, x):
        c1 = self.conv1(x)
        act1 = self.relu1(c1)
        out = self.relu2(self.shortcut_conv(x) + self.conv2(act1))
        return out