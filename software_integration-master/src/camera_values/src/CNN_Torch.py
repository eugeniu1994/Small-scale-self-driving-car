#!/usr/bin/env python

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from torch.autograd import Variable
import os
from torch.utils.data.dataset import Dataset
import cv2 as cv
from torch import Tensor
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
import time, rospkg
import csv
from sklearn.model_selection import train_test_split

#this file is used for CNN training
#dataset is in camera_values/src/Images folder

BATCH_SIZE = 64
N_EPOCHS = 10
NUM_CLASSES = 7
IMG_SIZE = 32

# this class define a convolutional layer, with max pooling and batch normalization
class ConvNet(nn.Module):

    def __init__(self, in_c, out_c,
                 kernel_size,
                 padding_size='same',
                 pool_stride=2,
                 batch_norm=True):
        super(ConvNet, self).__init__()

        if padding_size == 'same':
            padding_size = kernel_size // 2
        self.conv = nn.Conv2d(in_c, out_c, kernel_size, padding=padding_size)
        self.max_pool2d = nn.MaxPool2d(pool_stride, stride=pool_stride)
        self.batch_norm = batch_norm
        self.batch_norm_2d = nn.BatchNorm2d(out_c)

    def forward(self, x):
        x = self.max_pool2d(nn.functional.leaky_relu(self.conv(x)))

        if self.batch_norm:
            return self.batch_norm_2d(x)
        else:
            return x

#this function is used to compute the output of CNN,
#in order to coonect the CNN with fully connected layers
#it serves as a flatten layer
def get_convnet_output_size(network, input_size=IMG_SIZE):
    input_size = input_size or IMG_SIZE

    if type(network) != list:
        network = [network]

    in_channels = network[0].conv.in_channels

    output = Variable(torch.ones(1, in_channels, input_size, input_size))
    output.require_grad = False
    for conv in network:
        output = conv.forward(output)

    return np.asscalar(np.prod(output.data.shape)), output.data.size()[2]

#this class defines the neural network architecture
#3 CNN layer, 2 fully connected, softmax operator
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = ConvNet(3, 150, kernel_size=5, padding_size=0)
        self.conv2 = ConvNet(150, 200, kernel_size=3, padding_size=0)
        self.conv3 = ConvNet(200, 300, kernel_size=3, padding_size=0)

        inp, _ = get_convnet_output_size([self.conv1,
                                          self.conv2,
                                          self.conv3],
                                         IMG_SIZE)
        self.fc1 = nn.Linear(inp, 50)
        self.fc2 = nn.Linear(50, NUM_CLASSES)

    def forward(self, x):
        x = self.conv3(self.conv2(self.conv1(x)))
        x = x.view(x.size(0), -1)

        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        return F.log_softmax(x)

#this class is used to prepare data for training
class GTSRB(Dataset):
    def __init__(self, rootpath, training=True):
        images, labels = [], []
        classes = [1, 17, 33, 34, 38, 39]
        #negative class is 0
        prefix = rootpath + '/0/'
        #add negative class
        for ims in os.listdir(str(prefix)):
            try:
                img_path = os.path.join(prefix, ims)
                img = cv.imread(img_path)
                img = cv.resize(img, (IMG_SIZE, IMG_SIZE)) / 255.0
                images.append((img))
                cls = 6
                labels.append(cls)
            except:
                print("error at negative cls")
        #add all other classes from dataset
        for c in classes:
            prefix = rootpath + '/' + format(c, '05d') + '/'
            print("prefix: {}".format(prefix))
            fileName = prefix + 'GT-' + format(c, '05d') + '.csv'
            #gtFile = open(fileName, newline='\n')
            gtFile = open(fileName)
            gtReader = csv.reader(gtFile, delimiter=';')
            next(gtReader)
            for row in gtReader:
                img = cv.imread(prefix + row[0], -1)
                img = cv.resize(img, (IMG_SIZE, IMG_SIZE)) / 255.0
                images.append((img))
                cls = int(row[7])

                if cls == 1:
                    cls = 0
                elif cls == 17:
                    cls = 1
                elif cls == 33:
                    cls = 2
                elif cls == 34:
                    cls = 3
                elif cls == 38:
                    cls = 4
                elif cls == 39:
                    cls = 5

                labels.append(cls)
            gtFile.close()

        #split images and labes, in 80% training and 20% testing
        x_train, x_test, y_train, y_test = train_test_split(images, labels, test_size=0.2)
        if training == True:
            data = [(x, y) for x, y in zip(x_train, y_train)]
        else:
            data = [(x, y) for x, y in zip(x_test, y_test)]
        self.data = data

    def __len__(self):
        return len(self.data)

    #used to comvert image to tensor for torch
    def __getitem__(self, index):
        img = self.data[index][0]
        img_tensor = Tensor(img).view(3, IMG_SIZE, IMG_SIZE).float()
        label = self.data[index][1]
        return (img_tensor, label)

rospack = rospkg.RosPack()
#path where to save the binary file
modelPath = os.path.join(rospack.get_path("camera_values"), "launch", "Traffic_GTSRB.pt")

#used to train
def callFunctionTrain():
    root = os.path.dirname(__file__)
    train_dataset = GTSRB(os.path.join(root, 'Images'),training=True) #get data for training
    print("Loaded data")
    train_loader = DataLoader(dataset=train_dataset, batch_size=BATCH_SIZE, shuffle=True)

    model = Net() #neural network

    #loss function crossentropy
    criterion = torch.nn.CrossEntropyLoss()

    #train with different optimisers, the best is SGD
    #optimizer = torch.optim.Adam(model.parameters())
    optimizer = torch.optim.SGD(model.parameters(), lr=0.01, momentum=0.9)
    #optimizer = torch.optim.RMSprop(model.parameters(), lr=0.01, alpha=0.99, eps=1e-08, weight_decay=0, momentum=0, centered=False)

    loss_history = []

    #define train function
    def train(epoch):
        epoch_loss = 0
        n_batches = len(train_dataset) // BATCH_SIZE

        for step, data in enumerate(train_loader, 0):
            train_x, train_y = data
            y_hat = model.forward(train_x)
            train_y = torch.Tensor(np.array(train_y))

            loss = criterion(y_hat, train_y.long())
            epoch_loss += loss.item()
            optimizer.zero_grad()

            loss.backward()
            optimizer.step()

            if step % n_batches == 0 and step != 0:
                epoch_loss = epoch_loss / n_batches
                loss_history.append(epoch_loss)
                print("Epoch {}, loss {}".format(epoch, epoch_loss))
                epoch_loss = 0

    #traininf for N_EPOCHS
    print("Start training")
    for epoch in range(1, N_EPOCHS + 1):
        train(epoch)

    #save model
    torch.save(model.state_dict(), modelPath)
    print("Saved model...")

    #print loss
    plt.plot(np.array(range(1, N_EPOCHS + 1)), loss_history)
    plt.xlabel('Iterations')
    plt.ylabel('Loss')
    plt.show()

#used for testing
def callFunctionTest():
    root = os.path.dirname(__file__)
    test_dataset = GTSRB(os.path.join(root, 'Images'), training=False)#get data for testing
    test_loader = DataLoader(test_dataset, batch_size=1, shuffle=True)

    #get model and load binary files from modelPath
    classifier = Net()
    classifier.load_state_dict(torch.load(modelPath))
    classifier.eval()

    #check the model for unseen data
    correct = 0
    for _, data in enumerate(test_loader, 0):
        test_x, test_y = data

        pred = classifier.forward(test_x)
        y_hat = np.argmax(pred.data)
        if y_hat == test_y[0]:
            correct += 1.0

    print("Accuracy={}, correct {}, len is {}".format(correct / len(test_dataset),correct,len(test_dataset)))


#call tran & test
callFunctionTrain()
callFunctionTest()

