import numpy as np
import cv2
import tensorflow as tf
from collections import Counter
import matplotlib.pyplot as plt

classes = {
    0: 'Stop',
    1: 'Road work',
    2: 'Turn right ahead',
    3: 'Turn left ahead',
    4: 'Ahead only',
}


def predict_image(model_path, image_path):
    # Load the model
    model = tf.keras.models.load_model(model_path)

    # Load and preprocess the image
    image = cv2.imread(image_path)
    image = cv2.resize(image, (30, 30))
    image = np.expand_dims(image, axis=0)
    image = image / 255.0

    predictions = model.predict(image)
    return np.argmax(predictions)


model_path = "C:/Users/Esteban/Documents/Escuela/Robotica/ROS/src/final_challenge/training/saved_model.h5"
image_path = "C:/Users/Esteban/Documents/Escuela/Robotica/ROS/src/final_challenge/training/test.jpg"
