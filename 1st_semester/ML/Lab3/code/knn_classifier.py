"""Implements a k-Nearest Neighbors (kNN) classifier."""

import numpy as np
from typing import Optional


class KNNClassifier:
    """k-Nearest Neighbors (kNN) classifier."""

    def __init__(self, k: int) -> None:
        """Initialize the kNN classifier.

        Parameters
        ----------
        k : int
            Number of neighbors to use.

        """
        if k <= 0:
            raise ValueError("k must be a positive integer.")
        self.k = k
        self.X_train: Optional[np.ndarray] = None
        self.y_train: Optional[np.ndarray] = None

    def fit(self, X_train: np.ndarray, y_train: np.ndarray) -> None:
        """Fit the kNN classifier with training data.

        Parameters
        ----------
        X_train : np.ndarray
            Training data (n_samples x n_features). The training samples.
        y_train : np.ndarray
            Training labels (n_samples,). The labels corresponding to the training samples.

        """
        if X_train.shape[0] != y_train.shape[0]:
            raise ValueError(
                f"The number of samples in X_train ({X_train.shape[0]}) "
                f"does not match the number of labels in y_train ({y_train.shape[0]})."
            )
        if self.k > X_train.shape[0]:
            raise ValueError(
                f"k ({self.k}) must not be greater than the number of training samples ({X_train.shape[0]})."
            )
        self.X_train = X_train
        self.y_train = y_train

    def predict(self, X_test: np.ndarray) -> np.ndarray:
        """Predict labels for test data.

        Parameters
        ----------
        X_test : np.ndarray
            Test data (n_samples x n_features).
            The data to predict labels for.
            Each row represents a sample, and each column represents a feature.

        Returns
        -------
        np.ndarray: Predicted labels for the test data.

        """
        if self.X_train is None or self.y_train is None:
            raise ValueError("The classifier has not been fitted yet.")
        predictions = []

        # Classify each test sample
        for test_point in X_test:
            # Compute distances to all training samples
            distances = np.linalg.norm(self.X_train - test_point, axis=1)
            # Get the indices of the k nearest neighbors
            nearest_indices = np.argsort(distances)[: self.k]
            nearest_labels = self.y_train[nearest_indices]
            # Majority vote
            unique_labels, counts = np.unique(nearest_labels, return_counts=True)
            predicted_label = unique_labels[np.argmax(counts)]
            predictions.append(predicted_label)

        return np.array(predictions)


def knn_classifier(*args):
    """k-Nearest Neighbors (kNN) classifier.

    Parameters
    ----------
    *args : tuple
        Variable length argument list. Expected arguments are:
        - X_train (np.ndarray): Training data.
        - y_train (np.ndarray): Training labels.
        - X_test (np.ndarray): Test data.
        - k (int): Number of neighbors to use.
        - y_test (np.ndarray, optional): True labels for the test set.

    Returns
    -------
    - predictions (np.ndarray): Predicted labels for the test set.
    - error_rate (float, optional): Error rate if y_test is provided.

    """
    # Check number of arguments
    if len(args) not in [4, 5]:
        raise ValueError(f"Expected 4 or 5 arguments, but got {len(args)}.")

    X_train, y_train, X_test, k = args[:4]
    y_test = args[4] if len(args) == 5 else None

    # Use KNNClassifier
    knn = KNNClassifier(k)
    knn.fit(X_train, y_train)
    predictions = knn.predict(X_test)

    if y_test is not None:
        error_rate = np.mean(predictions != y_test)
        return predictions, error_rate

    return predictions
