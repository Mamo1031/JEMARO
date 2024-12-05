"""Module for training and visualizing an autoencoder on the MNIST dataset."""

import tensorflow as tf
from tensorflow.keras.datasets import mnist
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, Flatten, Reshape
from sklearn.manifold import TSNE
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple


class MNISTClassifier:
    """A class for training and visualizing an autoencoder on the MNIST dataset.

    Attributes:
        latent_dim (int): The size of the latent representation in the autoencoder.
        model (Model): The overall model (not used in this implementation).
        autoencoder (Model): The full autoencoder model.
        encoder (Model): The encoder part of the autoencoder.
        decoder (Model): The decoder part of the autoencoder.
        history: The training history object from Keras.

    """

    def __init__(self, latent_dim: int = 32) -> None:
        """Initialize the MNISTClassifier with the given latent dimension size.

        Args:latent_dim (int): The size of the latent space in the autoencoder.
        """
        self.latent_dim = latent_dim
        self.model = None
        self.autoencoder = None
        self.encoder = None
        self.decoder = None
        self.history = None

    def load_data(
        self,
    ) -> Tuple[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]:
        """Load and preprocess the MNIST dataset.

        Returns:
            Tuple[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]:
                The training and testing data as tuples of (data, labels).

        """
        (x_train, y_train), (x_test, y_test) = mnist.load_data()
        x_train = x_train.astype("float32") / 255.0
        x_test = x_test.astype("float32") / 255.0
        x_train = np.expand_dims(x_train, axis=-1)
        x_test = np.expand_dims(x_test, axis=-1)
        y_train = tf.keras.utils.to_categorical(y_train, 10)
        y_test = tf.keras.utils.to_categorical(y_test, 10)
        return (x_train, y_train), (x_test, y_test)

    def build_autoencoder(self) -> None:
        """Build the autoencoder, including encoder and decoder components."""
        input_shape = (28, 28, 1)
        input_layer = Input(shape=input_shape)
        x = Flatten()(input_layer)
        latent = Dense(self.latent_dim, activation="relu")(x)

        # Encoder and decoder
        self.encoder = Model(input_layer, latent, name="encoder")

        latent_input = Input(shape=(self.latent_dim,))
        decoder_output = Dense(np.prod(input_shape), activation="sigmoid")(latent_input)
        decoder_output = Reshape(input_shape)(decoder_output)
        self.decoder = Model(latent_input, decoder_output, name="decoder")

        # Autoencoder
        autoencoder_output = self.decoder(self.encoder(input_layer))
        self.autoencoder = Model(input_layer, autoencoder_output, name="autoencoder")

        self.autoencoder.compile(optimizer="adam", loss="mse")
        print(self.autoencoder.summary())

    def train_autoencoder(
        self, x_train: np.ndarray, epochs: int = 10, batch_size: int = 128
    ) -> None:
        """Trains the autoencoder on the training data.

        Args:
            x_train (np.ndarray): The training data.
            epochs (int): The number of epochs for training.
            batch_size (int): The batch size for training.

        """
        self.history = self.autoencoder.fit(
            x_train, x_train, epochs=epochs, batch_size=batch_size, validation_split=0.2
        )

    def visualize_latent_space(self, x_test: np.ndarray, y_test: np.ndarray) -> None:
        """Visualizes the latent space using t-SNE.

        Args:
            x_test (np.ndarray): The test data.
            y_test (np.ndarray): The test labels (one-hot encoded).

        """
        latent_representations = self.encoder.predict(x_test)

        # Limit sample size
        num_samples = 1000  # Maximum Samples
        indices = np.random.choice(
            latent_representations.shape[0], num_samples, replace=False
        )
        sampled_latent = latent_representations[indices]
        sampled_labels = y_test[indices]

        # Calculating t-SNE
        tsne = TSNE(n_components=2, random_state=42, perplexity=30)
        latent_2d = tsne.fit_transform(sampled_latent)

        # Plotting
        plt.figure(figsize=(8, 8))
        for i in range(10):
            idx = np.argmax(sampled_labels, axis=1) == i
            plt.scatter(latent_2d[idx, 0], latent_2d[idx, 1], label=str(i), alpha=0.5)
        plt.legend()
        plt.title("Latent Space Visualization (t-SNE)")
        plt.xlabel("t-SNE Dimension 1")
        plt.ylabel("t-SNE Dimension 2")
        plt.grid()
        plt.show()

    def visualize_reconstruction(self, x_test: np.ndarray) -> None:
        """Visualizes the original and reconstructed images.

        Args:x_test (np.ndarray): The test data.
        """
        reconstructions = self.autoencoder.predict(x_test[:10])

        plt.figure(figsize=(10, 4))
        for i in range(10):
            # Original image
            plt.subplot(2, 10, i + 1)
            plt.imshow(x_test[i].squeeze(), cmap="gray")
            plt.axis("off")
            # Reconstructed image
            plt.subplot(2, 10, i + 11)
            plt.imshow(reconstructions[i].squeeze(), cmap="gray")
            plt.axis("off")
        plt.suptitle("Original Images (Top) vs Reconstructed Images (Bottom)")
        plt.show()
