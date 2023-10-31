# Importing the NumPy library for numerical computations
import numpy as np
# Importing the Pandas library for data manipulation and analysis
import pandas as pd
# Importing the os module for interacting with the operating system
import os
import cv2                         # Importing the OpenCV library for image processing
import warnings
# Importing the Matplotlib library for data visualization
import matplotlib.pyplot as plt
# Importing the TensorFlow library for deep learning
import tensorflow as tf
# Importing the Keras module from TensorFlow for building neural networks
from tensorflow import keras
from PIL import Image              # Importing the PIL library for image processing
from tensorflow.keras.optimizers import Adam
from keras.utils import to_categorical
# Importing train_test_split from scikit-learn for splitting data
from sklearn.model_selection import train_test_split
# Importing ImageDataGenerator for data augmentation
from tensorflow.keras.preprocessing.image import ImageDataGenerator
# Importing the Adam optimizer from TensorFlow for model optimization
from tensorflow.keras.optimizers import Adam
# Importing accuracy_score from scikit-learn for calculating accuracy
from sklearn.metrics import accuracy_score


def read_images(directory_path):
    images = []
    labels = []

    valid_extensions = ('.jpg', '.jpeg', '.png')  # Valid image file extensions

    # Get subdirectories within the directory
    subdirectories = [subdir for subdir in os.listdir(
        directory_path) if os.path.isdir(os.path.join(directory_path, subdir))]

    for subdir in subdirectories:
        subdirectory_path = os.path.join(directory_path, subdir)
        if not os.listdir(subdirectory_path):
            continue  # Skip empty subdirectories

        # Iterate over images in the subdirectory
        for filename in os.listdir(subdirectory_path):
            if filename.lower().endswith(valid_extensions):  # Check if the file has a valid image extension
                image_path = os.path.join(subdirectory_path, filename)
                image = cv2.imread(image_path)
                # Resize the image to 30x30 pixels
                image = cv2.resize(image, (30, 30))
                images.append(image)

                # Assign the label based on the subdirectory name
                label = int(subdir)
                labels.append(label)

    data = np.array(list(zip(images, labels)), dtype=object)

    return data


train_data = read_images(
    'C:/Users/Esteban/Documents/Escuela/Robotica/ROS/src/final_challenge/training/archive/Train')

print(f"Train Data Shape : {train_data.shape}")

classes = {
    0: 'Stop',
    1: 'Road work',
    2: 'Turn right ahead',
    3: 'Turn left ahead',
    4: 'Ahead only',
}

labels = train_data[:, 1].astype(int)

# Get the unique labels and their counts
unique_labels, label_counts = np.unique(labels, return_counts=True)

# Create a copy of train_data to avoid modifying the original array
shuffled_train = train_data.copy()
shuffled_train = np.array(shuffled_train)  # Shuffle the array randomly

# Split the shuffled_train array into training and testing sets
train_set, test_set = train_test_split(
    shuffled_train, test_size=0.2, random_state=42)

# Separate the input (X) and output/label (y) arrays from the training set
x_train = train_set[:, 0]
y_train = train_set[:, 1]

# Separate the input (X) and output/label (y) arrays from the testing set
x_test = test_set[:, 0]
y_test = test_set[:, 1]

# Convert the data type of the arrays to int
x_train = np.array([np.array(x) for x in x_train])
y_train = np.array([np.array(y) for y in y_train])
x_test = np.array([np.array(x) for x in x_test])
y_test = np.array([np.array(y) for y in y_test])

# Normalize the input data
x_train = x_train / 255.0
x_test = x_test / 255.0

print("x_train.shape:", x_train.shape)
print("x_test.shape:", x_test.shape)
print("y_train.shape:", y_train.shape)
print("y_test.shape:", y_test.shape)
y_train = to_categorical(y_train, num_classes=5)
y_test = to_categorical(y_test, num_classes=5)

model = tf.keras.Sequential([
    tf.keras.layers.Conv2D(filters=16, kernel_size=(
        3, 3), activation='relu', input_shape=(30, 30, 3)),
    tf.keras.layers.Conv2D(filters=32, kernel_size=(3, 3), activation='relu'),
    tf.keras.layers.MaxPool2D(pool_size=(2, 2)),
    tf.keras.layers.BatchNormalization(axis=-1),

    tf.keras.layers.Conv2D(filters=64, kernel_size=(3, 3), activation='relu'),
    tf.keras.layers.Conv2D(filters=128, kernel_size=(3, 3), activation='relu'),
    tf.keras.layers.MaxPool2D(pool_size=(2, 2)),
    tf.keras.layers.BatchNormalization(axis=-1),

    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(512, activation='relu'),
    tf.keras.layers.BatchNormalization(),
    tf.keras.layers.Dropout(rate=0.5),

    tf.keras.layers.Dense(5, activation='softmax')
])

lr = 0.001
epochs = 30

opt = Adam(learning_rate=lr)  # Set the learning_rate instead of decay
model.compile(loss='categorical_crossentropy',
              optimizer=opt, metrics=['accuracy'])

aug = ImageDataGenerator(
    rotation_range=15,            # Increase the rotation range for more varied rotations
    zoom_range=0.2,               # Increase the zoom range for more varied zoom levels
    width_shift_range=0.15,       # Increase the range of horizontal shift
    height_shift_range=0.15,      # Increase the range of vertical shift
    shear_range=0.2,              # Increase the shear range for more shearing effects
    horizontal_flip=True,         # Enable horizontal flipping
    vertical_flip=True,           # Enable vertical flipping
    fill_mode="reflect"           # Use reflect mode for filling empty pixels
)


history = model.fit(aug.flow(x_train, y_train, batch_size=32),
                    epochs=epochs, validation_data=(x_test, y_test))

plt.figure(figsize=(10, 5))
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.legend()
plt.show()

# Plot training and validation accuracy
plt.figure(figsize=(10, 5))
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.title('Model Accuracy')
plt.xlabel('Epochs')
plt.ylabel('Accuracy')
plt.legend()
plt.show()

shuffled_train_dataframe = pd.DataFrame(shuffled_train)
# Save DataFrame as CSV file
shuffled_train_dataframe.to_csv('data.csv', index=False)

model.save("saved_model.h5")
