import numpy as np



class NaiveBayesClassifier:
    def __init__(self, alpha):
        self.alpha = alpha
        self.target_prob = {}
        self.feature_prob = {}


    def fit(self, train_data, target_column):
        """
        Fit the Naive Bayes model to the training data.

        Parameters:
            train_data (DataFrame): Training data
            target_column (str): Target column name
        """
        target_counts = train_data[target_column].value_counts().to_dict()
        total_train = len(train_data)
        self.target_prob = {k: v / total_train for k, v in target_counts.items()}
        
        features = train_data.columns.drop(target_column)
        self.feature_prob = {}
        
        for feature in features:
            self.feature_prob[feature] = {}
            feature_values = train_data[feature].unique()
            v = len(feature_values)
            for value in feature_values:
                self.feature_prob[feature][value] = {}
                for cls in target_counts:
                    count = len(train_data[(train_data[feature] == value) & (train_data[target_column] == cls)])
                    class_count = target_counts[cls]
                    prob = (count + self.alpha) / (class_count + self.alpha * v) if class_count > 0 else 0
                    self.feature_prob[feature][value][cls] = prob


    def predict(self, test_data, target_column=None):
        """
        Predict the class for the test data.

        Parameters:
            test_data (DataFrame): Test data
            target_column (str, optional): Target column name in the test data

        Returns:
            predictions (list): List of predicted class labels
        """
        predictions = []
        for _, row in test_data.iterrows():
            class_scores = {}
            for cls in self.target_prob:
                score = self.target_prob[cls]
                for feature in self.feature_prob:
                    value = row[feature]
                    score *= self.feature_prob[feature].get(value, {}).get(cls, 0)
                class_scores[cls] = score
            predicted_class = max(class_scores, key=class_scores.get)
            predictions.append(predicted_class)

        return predictions


    def error_rate(self, test_data, predictions, target_column):
        """
        Calculate the error rate of the predictions.

        Parameters:
            test_data (DataFrame): Test data
            predictions (list): List of predicted class labels
            target_column (str): Target column name

        Returns:
            float or None: Error rate, or None if target_column is missing from test_data
        """
        if target_column not in test_data.columns:
            print(f"Warning: Target column '{target_column}' is missing from the test data. Cannot calculate error rate.")
            return None
        
        actual = test_data[target_column].values
        error_count = sum(1 for a, p in zip(actual, predictions) if a != p)

        return error_count / len(test_data)



class NaiveBayesClassifierWithLog(NaiveBayesClassifier):
    def fit(self, train_data, target_column):
        """
        Fit the Naive Bayes model with log probabilities.
        """
        target_counts = train_data[target_column].value_counts().to_dict()
        total_train = len(train_data)
        self.target_prob = {k: np.log(v / total_train) for k, v in target_counts.items()}  # 
        
        features = train_data.columns.drop(target_column)
        self.feature_prob = {}
        
        for feature in features:
            self.feature_prob[feature] = {}
            feature_values = train_data[feature].unique()
            v = len(feature_values)
            for value in feature_values:
                self.feature_prob[feature][value] = {}
                for cls in target_counts:
                    count = len(train_data[(train_data[feature] == value) & (train_data[target_column] == cls)])
                    class_count = target_counts[cls]
                    prob = (count + self.alpha) / (class_count + self.alpha * v) if class_count > 0 else 0
                    self.feature_prob[feature][value][cls] = np.log(prob) if prob > 0 else -np.inf  # take the logarithm


    def predict(self, test_data, target_column=None):
        """
        Predict the class for the test data using log probabilities.
        """
        predictions = []
        for _, row in test_data.iterrows():
            class_scores = {}
            for cls in self.target_prob:
                score = self.target_prob[cls]  # log prior probabilities
                for feature in self.feature_prob:
                    value = row[feature]
                    score += self.feature_prob[feature].get(value, {}).get(cls, -np.inf)  # summing log conditional probabilities
                class_scores[cls] = score
            predicted_class = max(class_scores, key=class_scores.get)
            predictions.append(predicted_class)
            
        return predictions
