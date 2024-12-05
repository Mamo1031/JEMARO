"""Execute the MNIST autoencoder task.

It involves:
1. Loading MNIST data.
2. Building and training an autoencoder.
3. Visualizing the latent space and reconstructed images.
"""

from mnist_classifier import MNISTClassifier


def main() -> None:
    """Execute the MNIST autoencoder task.

    It involves:
    1. Loading MNIST data.
    2. Building and training an autoencoder.
    3. Visualizing the latent space and reconstructed images.

    Returns:None
    """
    # Define the latent dimension size
    latent_dim: int = 32

    # Initialize the MNIST classifier with the specified latent dimension
    classifier: MNISTClassifier = MNISTClassifier(latent_dim=latent_dim)

    # Load the MNIST dataset
    (x_train, y_train), (x_test, y_test) = classifier.load_data()

    # Build and train the autoencoder
    classifier.build_autoencoder()
    classifier.train_autoencoder(x_train, epochs=10, batch_size=128)

    # Visualize the latent space
    classifier.visualize_latent_space(x_test, y_test)

    # Visualize the original and reconstructed images
    classifier.visualize_reconstruction(x_test)


if __name__ == "__main__":
    main()
