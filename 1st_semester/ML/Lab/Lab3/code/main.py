"""The module provides a main entry point to run the kNN classifier evaluation.

It supports two datasets: 'MNIST' and 'Wine', allowing for testing the classifier
with different hyperparameters and providing performance metrics.
"""

from dataset import MNISTDataset, WineDataset
from evaluator import Evaluator


def main():
    """Run the kNN classifier evaluation."""
    # Prompt user to select the dataset
    print("Select dataset:")
    print("0: MNIST")
    print("1: Wine")

    try:
        choice = int(input("Enter 0 for MNIST or 1 for Wine: ").strip())
        if choice == 0:
            dataset_name = "MNIST"
            dataset = MNISTDataset(train_subset_size=6000, test_subset_size=1000)
        elif choice == 1:
            dataset_name = "Wine"
            dataset = WineDataset()
        else:
            raise ValueError("Invalid choice. Please enter 0 or 1.")
    except ValueError as e:
        print(f"Error: {e}")
        return

    # Load and preprocess the dataset
    dataset.load_data()
    dataset.preprocess()
    X_train, y_train, X_test, y_test = dataset.get_data()

    # Define k values to test
    ks = [1, 2, 3, 4, 5, 10, 15, 20, 30, 40, 50]

    # Initialize the evaluator
    evaluator = Evaluator(dataset_name=dataset_name, ks=ks)

    # Evaluate the classifier
    evaluator.evaluate(X_train, y_train, X_test, y_test)


if __name__ == "__main__":
    main()
