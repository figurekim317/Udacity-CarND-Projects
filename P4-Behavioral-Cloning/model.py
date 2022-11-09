import csv
import numpy as np
import sklearn
import cv2
from math import ceil
from sklearn.model_selection import train_test_split

from keras.models import Sequential
from keras.layers import Lambda, Cropping2D, Conv2D, Flatten, Dense, Dropout
import tensorflow as tf


#################
# Loading Dataset
#################
def load_dataset(paths=['./data/data_default/', './data/data_basic/', './data/data_reverse/']):
    samples = []
    for data_path in paths:
        with open(data_path + '/driving_log.csv') as csvfile:
            reader = csv.reader(csvfile)
            next(reader) # Skipping header
            for line in reader:
                # Loading center image and steering measurement
                center_img_path = data_path + line[0].strip()
                measurement = float(line[3])
                # Loading left image
                left_img_path = data_path + line[1].strip()
                # Loading right image
                right_img_path = data_path + line[2].strip()

                samples.append([center_img_path, left_img_path, right_img_path, measurement])
    
    return samples

#################
# Data Distribution Flattening
#################
def flatten_distribution(samples, num_bin=23):
    angles = np.array([sample[3] for sample in samples])
    avg_per_bin = len(angles)/num_bin

    hist, bins = np.histogram(angles, num_bin)
    keep_probs=[]

    # set keep probability for each bin
    for i in range(num_bin):
        if hist[i] < avg_per_bin:
            keep_probs.append(1.)
        else:
            keep_probs.append(1.*avg_per_bin/hist[i])
        
    # drop samples above average
    flatten_samples = []
    for sample in samples:
        for j in range(num_bin):
            if sample[3] > bins[j] and sample[3] <= bins[j+1]:
                if np.random.rand() <= keep_probs[j]:
                    flatten_samples.append(sample)

    return flatten_samples

#################
# Data Augumentation
#################
def augument_sample_and_split(samples, steering_bias=0.23):

    augumented_samples = []
    for sample in samples:
        center = [sample[0], sample[3], False]
        left = [sample[1], sample[3]+steering_bias, False]
        right = [sample[2], sample[3]-steering_bias, False]
        center_flip = [sample[0], -sample[3], True]
        left_flip = [sample[1], -(sample[3]+steering_bias), True]
        right_flip = [sample[2], -(sample[3]-steering_bias), True]
        augumented_samples.append(center)
        augumented_samples.append(left)
        augumented_samples.append(right)
        augumented_samples.append(center_flip)
        augumented_samples.append(left_flip)
        augumented_samples.append(right_flip)
    
    train_samples, validation_samples = train_test_split(augumented_samples, test_size=0.2)
    return train_samples, validation_samples

#################
# Model
#################
def nvidia_model():
    model = Sequential()
    # Preprocess incoming data.
    # Crop 50 rows pixels from the top, 20 rows pixels from the bottom
    # Centered around zero within range of [-1, 1]
    model.add(Cropping2D(cropping=((50,20), (0,0)), input_shape=(160,320,3)))
    model.add(Lambda(lambda x: (x/127.5 - 1.)))
    model.add(Lambda(lambda x: tf.image.resize_images(x, (66,200))))
    model.add(Conv2D(24, (5,5), padding="valid", strides=(2,2), activation="relu"))
    model.add(Conv2D(36, (5,5), padding="valid", strides=(2,2), activation="relu"))
    model.add(Conv2D(48, (5,5), padding="valid", strides=(2,2), activation="relu"))
    model.add(Conv2D(64, (3,3), padding="valid", activation="relu"))
    model.add(Conv2D(64, (3,3), padding="valid", activation="relu"))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))

    model.summary()

    return model

#################
# Batch Generator
#################
def generator(samples, batch_size=32):
    num_samples = len(samples)
    # Epoch Loop
    while 1: # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)

        # Batch Loop
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            # Load image from each sample
            for batch_sample in batch_samples:
                image = cv2.imread(batch_sample[0])
                try:
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                except:
                    print(batch_sample[0])
                angle = float(batch_sample[1])
                is_fliped = batch_sample[2]
                
                if is_fliped:
                    image = cv2.flip(image, 1)
                images.append(image)
                angles.append(angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

#################
# Training
#################
if __name__=='__main__':
    # load samples
    samples = load_dataset()
    samples = flatten_distribution(samples)
    train_samples, validation_samples = augument_sample_and_split(samples)
    # load network
    model = nvidia_model()

    # Set our batch size
    batch_size=32
    epochs=5

    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=batch_size)
    validation_generator = generator(validation_samples, batch_size=batch_size)
    model.compile(loss='mse', optimizer='adam')

    history_object = model.fit_generator(train_generator, \
                steps_per_epoch=ceil(len(train_samples)/batch_size), \
                validation_data=validation_generator, \
                validation_steps=ceil(len(validation_samples)/batch_size), \
                epochs=epochs, verbose=1)

    model.save('model.h5')
    np.save('history_object.npy', history_object)

    ### print the keys contained in the history object
    print(history_object.history.keys())