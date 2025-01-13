"""Provides functions to compute the confusion matrix and evaluation metrics.

The evaluation metrics include precision, recall, and F1-score.
"""

import numpy as np
from typing import Tuple, List, Dict


def compute_confusion_matrix(
    y_true: np.ndarray, y_pred: np.ndarray, labels: List[int] = None
) -> Tuple[np.ndarray, List[int]]:
    """Compute the confusion matrix from true labels and predicted labels.

    Parameters
    ----------
    y_true : np.ndarray
        True labels.
    y_pred : np.ndarray
        Predicted labels.
    labels : List[int], optional
        List of class labels. If None, they will be inferred.

    Returns
    -------
    Tuple containing:
        - conf_matrix (np.ndarray): Confusion matrix.
        - class_labels (List[int]): List of class labels.

    """
    if labels is None:
        class_labels = np.unique(np.concatenate((y_true, y_pred)))
    else:
        class_labels = labels

    label_to_index = {label: index for index, label in enumerate(class_labels)}

    conf_matrix = np.zeros((len(class_labels), len(class_labels)), dtype=int)

    for true_label, pred_label in zip(y_true, y_pred):
        i = label_to_index[true_label]
        j = label_to_index[pred_label]
        conf_matrix[i][j] += 1

    return conf_matrix, class_labels


def compute_metrics(conf_matrix: np.ndarray) -> Dict[str, float]:
    """Compute precision, recall, and F1-score from the confusion matrix.

    Parameters
    ----------
    conf_matrix : np.ndarray
        Confusion matrix.

    Returns
    -------
    Dict[str, float]: Dictionary containing precision, recall, and F1-score.

    """
    precision_list = []
    recall_list = []
    f1_list = []
    num_classes = conf_matrix.shape[0]

    for i in range(num_classes):
        tp = conf_matrix[i, i]
        fp = conf_matrix[:, i].sum() - tp
        fn = conf_matrix[i, :].sum() - tp

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = (
            2 * precision * recall / (precision + recall)
            if (precision + recall) > 0
            else 0.0
        )

        precision_list.append(precision)
        recall_list.append(recall)
        f1_list.append(f1)

    # Compute macro-average
    precision = np.mean(precision_list)
    recall = np.mean(recall_list)
    f1_score = np.mean(f1_list)

    return {"precision": precision, "recall": recall, "f1_score": f1_score}
