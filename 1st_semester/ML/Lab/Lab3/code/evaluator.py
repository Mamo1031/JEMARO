"""Provides an Evaluator class to evaluate the kNN classifier on a given dataset."""

import numpy as np
import matplotlib.pyplot as plt
from typing import Any, List, Dict, Tuple
from knn_classifier import knn_classifier
from metrics import compute_confusion_matrix, compute_metrics


class Evaluator:
    """Class to evaluate the kNN classifier on a given dataset."""

    def __init__(self, dataset_name: str, ks: List[int]) -> None:
        """Initialize the evaluator.

        Parameters
        ----------
        dataset_name : str
            Name of the dataset ('MNIST' or 'Wine').
        ks : List[int]
            List of k values to test.

        """
        self.dataset_name = dataset_name
        self.ks = ks
        self.results: Dict[int, Dict[str, Any]] = {}
        self.metrics_summary: Dict[str, List[Tuple[float, float]]] = {
            "precision": [],
            "recall": [],
            "f1_score": [],
        }

    def evaluate(
        self,
        X_train: np.ndarray,
        y_train: np.ndarray,
        X_test: np.ndarray,
        y_test: np.ndarray,
    ) -> None:
        """Evaluate the kNN classifier on the dataset.

        Parameters
        ----------
        X_train : np.ndarray
            Training data.
        y_train : np.ndarray
            Training labels.
        X_test : np.ndarray
            Test data.
        y_test : np.ndarray
            Test labels.

        """
        num_classes = self._get_num_classes(y_train)

        for label in range(num_classes):
            print("------------------")
            print(f"Testing class {label}...")
            train_labels, test_labels, class_labels = self._get_labels(
                y_train, y_test, label
            )
            ks_used, label_results_accuracy, label_results_metrics = (
                self._evaluate_for_label(
                    X_train,
                    train_labels,
                    X_test,
                    test_labels,
                    class_labels,
                    num_classes,
                )
            )
            self.results[label] = {
                "ks": ks_used,
                "accuracy": label_results_accuracy,
                "metrics": label_results_metrics,
            }

        self._compute_average_metrics()
        self._plot_results(dataset_name=self.dataset_name)
        self._print_metrics_summary()

    def _get_num_classes(self, y_train: np.ndarray) -> int:
        if self.dataset_name == "MNIST":
            return 10
        elif self.dataset_name == "Wine":
            return len(np.unique(y_train))
        else:
            raise ValueError("Unsupported dataset. Choose 'MNIST' or 'Wine'.")

    def _get_labels(
        self, y_train: np.ndarray, y_test: np.ndarray, label: int
    ) -> Tuple[np.ndarray, np.ndarray, List[int]]:
        if self.dataset_name == "MNIST":
            # Binary classification: target class vs. others
            train_labels = (y_train == label).astype(int)
            test_labels = (y_test == label).astype(int)
            class_labels = [0, 1]
        else:
            # Multiclass classification
            train_labels = y_train
            test_labels = y_test
            class_labels = np.unique(y_train)
            # No need to iterate over classes in multiclass setting
            label = "All"
        return train_labels, test_labels, class_labels

    def _evaluate_for_label(
        self,
        X_train: np.ndarray,
        train_labels: np.ndarray,
        X_test: np.ndarray,
        test_labels: np.ndarray,
        class_labels: List[int],
        num_classes: int,
    ) -> Tuple[List[int], List[Dict[str, float]]]:
        """Evaluate the classifier for a specific class and return accuracies."""
        label_results_accuracy = []
        label_results_metrics = []
        ks_used = []
        for k in self.ks:
            # Skip if k is a multiple of the number of classes
            if k % num_classes == 0:
                print(
                    f"  Skipping k={k} because it is a multiple of the number of classes ({num_classes})."
                )
                continue

            print(f"  Testing k={k}...")
            predictions, error_rate = knn_classifier(
                X_train, train_labels, X_test, k, test_labels
            )

            accuracy = 1 - error_rate
            label_results_accuracy.append(accuracy)

            conf_matrix, _ = compute_confusion_matrix(
                test_labels, predictions, labels=class_labels
            )
            metrics = compute_metrics(conf_matrix)
            label_results_metrics.append(metrics)

            ks_used.append(k)

        return ks_used, label_results_accuracy, label_results_metrics

    def _compute_average_metrics(self) -> None:
        for label in self.results:
            precision_values = [
                res["precision"] for res in self.results[label]["metrics"]
            ]
            recall_values = [res["recall"] for res in self.results[label]["metrics"]]
            f1_values = [res["f1_score"] for res in self.results[label]["metrics"]]
            self.metrics_summary["precision"].append(
                (np.mean(precision_values), np.std(precision_values))
            )
            self.metrics_summary["recall"].append(
                (np.mean(recall_values), np.std(recall_values))
            )
            self.metrics_summary["f1_score"].append(
                (np.mean(f1_values), np.std(f1_values))
            )

    def _plot_results(self, dataset_name) -> None:
        """Plot results with accuracy for each class."""
        num_classes = len(self.results)
        if dataset_name == "MNIST":
            cols = 2
        elif dataset_name == "Wine":
            cols = 3
        rows = (num_classes + cols - 1) // cols  # Calculate rows needed

        fig, axes = plt.subplots(rows, cols, figsize=(15, 5 * rows))
        axes = axes.flatten()  # Flatten to iterate easily

        for idx, label in enumerate(self.results):
            ks_used = self.results[label]["ks"]
            accuracies = self.results[label]["accuracy"]

            ax = axes[idx]
            ax.plot(ks_used, accuracies, marker="o", label=f"Class {label}")
            ax.set_title(f"Class {label}")
            ax.set_xlabel("k (Number of Neighbors)")
            ax.set_ylabel("Accuracy")
            ax.grid()

        # Hide unused subplots
        for idx in range(len(self.results), len(axes)):
            axes[idx].axis("off")

        plt.tight_layout()
        plt.show()

    def _print_metrics_summary(self) -> None:
        print("----------------------------")
        print("----------------------------")
        for metric, values in self.metrics_summary.items():
            print(f"Summary of {metric.capitalize()} ({self.dataset_name}):")
            for idx, (mean_val, std_val) in enumerate(values):
                label = list(self.results.keys())[idx]
                print(f"  Class {label}: Mean={mean_val:.4f}, Std={std_val:.4f}")
            print("----------------------------")
