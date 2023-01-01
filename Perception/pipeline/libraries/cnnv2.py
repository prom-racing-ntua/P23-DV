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

class BasicCNN(nn.Module):
    def __init__(self, output_size = 14):
        super(BasicCNN, self).__init__()

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