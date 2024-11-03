import numpy as np
import matplotlib.pyplot as plt



class Evaluator:
    """
    A class for evaluating linear regression models with various metrics and visualization options.

    Attributes:
        model (LinearRegressionModel): The linear regression model to evaluate.
    """
    def __init__(self, model):
        """
        Initializes the Evaluator with a linear regression model.

        Parameters:
            model (LinearRegressionModel): An instance of the linear regression model to be evaluated.
        """
        self.model = model


    def fit_and_plot_random_subsets(self, X, y, subset_ratio=0.1):
        """
        Fits the model on subsets taken from the beginning and end of the data
        and plots the predictions for comparison.

        Parameters:
            X (np.ndarray): Feature matrix.
            y (np.ndarray): Target vector.
            subset_ratio (float): Proportion of data in each subset. Default is 0.1 (10%).
        """
        subset_size = int(len(X) * subset_ratio)
    
        X_start, y_start = X[:subset_size], y[:subset_size]
        X_end, y_end = X[-subset_size:], y[-subset_size:]
        
        self.model.fit(X_start, y_start)
        y_pred_start = self.model.predict(X)
        plt.plot(X, y_pred_start, label="Start subset")
        
        self.model.fit(X_end, y_end)
        y_pred_end = self.model.predict(X)
        plt.plot(X, y_pred_end, label="End subset")
        
        plt.scatter(X, y, color="blue", label="Original data")
        plt.title("Start and End Subset Comparison")
        plt.xlabel("S&P500")
        plt.ylabel("MSCI")
        plt.legend()
        plt.show()


    def test_model(self, X, y, test_ratio=0.05):
        """
        Tests the model by splitting the data into training and testing sets, 
        fitting the model, and computing MSE for both sets.

        Parameters:
            X (np.ndarray): Feature matrix.
            y (np.ndarray): Target vector.
            test_ratio (float): Ratio of data to use for testing. Default is 0.05 (5%).

        Returns:
            tuple: Fitted model parameters (theta), train MSE, test MSE.
        """
        test_size = int(len(X) * test_ratio)
        X_train, X_test = X[:-test_size], X[-test_size:]
        y_train, y_test = y[:-test_size], y[-test_size:]
        
        theta = self.model.fit(X_train, y_train)
        y_train_pred = self.model.predict(X_train)
        y_test_pred = self.model.predict(X_test)
        train_mse = self.model.mean_squared_error(y_train, y_train_pred)
        test_mse = self.model.mean_squared_error(y_test, y_test_pred)
        
        print(f"Train MSE: {train_mse}")
        print(f"Test MSE: {test_mse}")
        
        return theta, train_mse, test_mse


    def evaluate_on_random_splits(self, X, y, num_trials=10, split_ratio=0.05):
        """
        Evaluates the model on multiple random splits with a specified test ratio, 
        and plots the resulting MSE values for train and test sets.

        Parameters:
            X (np.ndarray): Feature matrix.
            y (np.ndarray): Target vector.
            num_trials (int): Number of random train/test splits. Default is 10.
            split_ratio (float): Proportion of data used for testing. Default is 0.05 (5%).
        """
        train_mse_list = []
        test_mse_list = []

        for _ in range(num_trials):
            _, train_mse, test_mse = self.test_model(X, y, test_ratio=split_ratio)
            train_mse_list.append(train_mse)
            test_mse_list.append(test_mse)

        plt.plot(range(num_trials), train_mse_list, label="Train MSE")
        plt.plot(range(num_trials), test_mse_list, label="Test MSE")
        plt.xlabel("Trial")
        plt.ylabel("Mean Squared Error")
        plt.title("Train and Test MSE on Multiple Random Splits (5% Test Data)")
        plt.legend()
        plt.show()
        
        print(f"Average Train MSE: {np.mean(train_mse_list)}")
        print(f"Average Test MSE: {np.mean(test_mse_list)}")
