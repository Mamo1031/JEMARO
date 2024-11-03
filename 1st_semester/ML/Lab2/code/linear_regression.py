import numpy as np



class LinearRegressionModel:
    """
    A class representing a simple linear regression model.

    Attributes:
        theta (np.ndarray): Model coefficients after fitting.
    """
    def __init__(self, fit_intercept):
        """
        Initializes the LinearRegressionModel with or without intercept.
        
        Parameters:
            fit_intercept (bool): Whether to fit an intercept term. Default is True.
        """
        self.theta = None
        self.fit_intercept = fit_intercept

    def fit(self, X, y):
        """
        Fits the linear regression model to the provided data using the normal equation.

        Parameters:
            X (np.ndarray): Feature matrix.
            y (np.ndarray): Target vector.

        Returns:
            np.ndarray: Fitted model parameters (theta).
        """
        if self.fit_intercept:
            X_b = np.c_[np.ones((len(X), 1)), X]  # Add bias term (intercept)
        else:
            X_b = X  # No intercept term
        self.theta = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(y)

        return self.theta


    def predict(self, X):
        """
        Predicts target values based on fitted model parameters.

        Parameters:
            X (np.ndarray): Feature matrix.

        Returns:
            np.ndarray: Predicted target values.
        """
        if self.fit_intercept:
            X_b = np.c_[np.ones((len(X), 1)), X]  # Add bias term (intercept)
        else:
            X_b = X  # No intercept term
            
        return X_b.dot(self.theta)


    def mean_squared_error(self, y_true, y_pred):
        """
        Computes mean squared error between actual and predicted target values.

        Parameters:
            y_true (np.ndarray): Actual target values.
            y_pred (np.ndarray): Predicted target values.

        Returns:
            float: Mean squared error.
        """
        return np.mean((y_true - y_pred) ** 2)
