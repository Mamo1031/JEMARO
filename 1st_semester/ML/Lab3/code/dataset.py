"""Module containing dataset classes for MNIST and Wine datasets."""

import numpy as np
from sklearn.datasets import load_wine
from tensorflow.keras.datasets import mnist
from typing import Tuple


class Dataset:
    """Base class for datasets."""

    def load_data(self) -> None:
        """Load the dataset."""
        raise NotImplementedError(
            "The load_data method must be implemented by subclasses."
        )

    def preprocess(self) -> None:
        """Preprocess the dataset."""
        raise NotImplementedError(
            "The preprocess method must be implemented by subclasses."
        )

    def get_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get the training and test data and labels.

        Returns:
            Tuple containing:
                - X_train (np.ndarray): Training data.
                - y_train (np.ndarray): Training labels.
                - X_test (np.ndarray): Test data.
                - y_test (np.ndarray): Test labels.

        """
        raise NotImplementedError(
            "The get_data method must be implemented by subclasses."
        )


class MNISTDataset(Dataset):
    """MNIST dataset class."""

    def __init__(
        self,
        train_subset_size: int = None,
        test_subset_size: int = None,
        random_state: int = 42,
    ) -> None:
        """Initialize the MNISTDataset with optional subset sizes.

        Args:
            train_subset_size (int, optional): Number of training samples to use. Defaults to None (use all).
            test_subset_size (int, optional): Number of test samples to use. Defaults to None (use all).
            random_state (int): Seed for random number generator.

        """
        self.X_train: np.ndarray = np.array([])
        self.y_train: np.ndarray = np.array([])
        self.X_test: np.ndarray = np.array([])
        self.y_test: np.ndarray = np.array([])
        self.train_subset_size = train_subset_size
        self.test_subset_size = test_subset_size
        self.random_state = random_state

    def load_data(self) -> None:
        """Load the MNIST dataset."""
        (self.X_train, self.y_train), (self.X_test, self.y_test) = mnist.load_data()

    def preprocess(self) -> None:
        """Preprocess the MNIST dataset."""
        # Flatten the images
        self.X_train = self.X_train.reshape(self.X_train.shape[0], -1)
        self.X_test = self.X_test.reshape(self.X_test.shape[0], -1)
        # Normalize to [0, 1]
        self.X_train = self.X_train / 255.0
        self.X_test = self.X_test / 255.0

        # Use subsets if specified
        if self.train_subset_size is not None:
            np.random.seed(self.random_state)
            train_indices = np.random.choice(
                self.X_train.shape[0], self.train_subset_size, replace=False
            )
            self.X_train = self.X_train[train_indices]
            self.y_train = self.y_train[train_indices]

        if self.test_subset_size is not None:
            np.random.seed(self.random_state)
            test_indices = np.random.choice(
                self.X_test.shape[0], self.test_subset_size, replace=False
            )
            self.X_test = self.X_test[test_indices]
            self.y_test = self.y_test[test_indices]

    def get_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get the preprocessed training and test data and labels.

        Returns:
            Tuple containing:
                - X_train (np.ndarray): Training data.
                - y_train (np.ndarray): Training labels.
                - X_test (np.ndarray): Test data.
                - y_test (np.ndarray): Test labels.

        """
        return self.X_train, self.y_train, self.X_test, self.y_test


class WineDataset(Dataset):
    """Wine dataset class."""

    def __init__(self, train_ratio: float = 0.8, random_state: int = 42) -> None:
        """Initialize the WineDataset with optional train ratio and random state.

        Args:
            train_ratio (float): Ratio of training data to total data.
            random_state (int): Seed for random number generator.

        """
        self.X: np.ndarray = np.array([])
        self.y: np.ndarray = np.array([])
        self.X_train: np.ndarray = np.array([])
        self.y_train: np.ndarray = np.array([])
        self.X_test: np.ndarray = np.array([])
        self.y_test: np.ndarray = np.array([])
        self.train_ratio = train_ratio
        self.random_state = random_state

    def load_data(self) -> None:
        """Load the Wine dataset."""
        wine = load_wine()
        self.X, self.y = wine.data, wine.target

    def preprocess(self) -> None:
        """Preprocess the Wine dataset."""
        # Normalize the data to [0, 1]
        self.X = (self.X - self.X.min(axis=0)) / (
            self.X.max(axis=0) - self.X.min(axis=0)
        )
        # Split into training and test sets
        n_samples = self.X.shape[0]
        n_train = int(n_samples * self.train_ratio)
        np.random.seed(self.random_state)
        indices = np.random.permutation(n_samples)
        train_indices = indices[:n_train]
        test_indices = indices[n_train:]
        self.X_train, self.X_test = self.X[train_indices], self.X[test_indices]
        self.y_train, self.y_test = self.y[train_indices], self.y[test_indices]

    def get_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get the preprocessed training and test data and labels.

        Returns:
            Tuple containing:
                - X_train (np.ndarray): Training data.
                - y_train (np.ndarray): Training labels.
                - X_test (np.ndarray): Test data.
                - y_test (np.ndarray): Test labels.

        """
        return self.X_train, self.y_train, self.X_test, self.y_test
