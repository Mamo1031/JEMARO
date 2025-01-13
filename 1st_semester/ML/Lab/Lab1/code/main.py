from data_preprocessing import DataPreprocessor
from naive_bayes import NaiveBayesClassifier, NaiveBayesClassifierWithLog



# Task 1: Data preprocessing
weather_file_path = '../data/weather_data.csv'
weather_data_preprocessor = DataPreprocessor(weather_file_path, separator='\s+')
weather_data = weather_data_preprocessor.load_data()
weather_data_train, weather_data_test = weather_data_preprocessor.split_data(weather_data)
weather_target_column = weather_data_train.columns[-1]


# Task 2: Build a naive Bayes classifier
nb_classifier = NaiveBayesClassifier(alpha=0)
nb_classifier.fit(weather_data_train, weather_target_column)
predictions = nb_classifier.predict(weather_data_test)

result_df = weather_data_test.copy()
result_df['Prediction'] = predictions
print("---------------------------------------------------------------------------")
print("\nTest Data and Prediction results of the Naive Bayes classifier:")
print(result_df)
error_rate = nb_classifier.error_rate(weather_data_test, predictions, weather_target_column)
if error_rate is not None:
    print(f"\nNaive Bayes Classifier (without Laplace smoothing) error rate: {error_rate:.2f}")
else:
    print("\nError rate cannot be calculated because the test set does not contain the target column.")


# Task 3: Make the classifier robust to missing data with Laplace (additive) smoothing
nb_classifier_laplace = NaiveBayesClassifier(alpha=1)
nb_classifier_laplace.fit(weather_data_train, weather_target_column)
smoothed_predictions = nb_classifier_laplace.predict(weather_data_test)

smoothed_result_df = weather_data_test.copy()
smoothed_result_df['Prediction'] = smoothed_predictions
print("---------------------------------------------------------------------------")
print("\nTest Data and Prediction results of the Naive Bayes classifier with Laplace smoothing:")
print(smoothed_result_df)
smoothed_error_rate = nb_classifier_laplace.error_rate(weather_data_test, smoothed_predictions, weather_target_column)
if smoothed_error_rate is not None:
    print(f"\nNaive Bayes Classifier (with Laplace smoothing) error rate: {smoothed_error_rate:.2f}")
else:
    print("\nError rate cannot be calculated because the test set does not contain the target column.")


# Think further: Naive Bayes Classifier with log probabilities
nb_classifier_log = NaiveBayesClassifierWithLog(alpha=1)
nb_classifier_log.fit(weather_data_train, weather_target_column)
log_predictions = nb_classifier_log.predict(weather_data_test)

log_result_df = weather_data_test.copy()
log_result_df['Prediction'] = log_predictions
print("---------------------------------------------------------------------------")
print("\nTest Data and Prediction results of the Naive Bayes classifier with log probabilities:")
print(log_result_df)
log_error_rate = nb_classifier_log.error_rate(weather_data_test, log_predictions, weather_target_column)
if log_error_rate is not None:
    print(f"\nNaive Bayes Classifier (with log probabilities) error rate: {log_error_rate:.2f}")
else:
    print("\nError rate cannot be calculated because the test set does not contain the target column.")


# Think further: Naive Bayes Classifier for continuous data
iris_file_path = '../data/iris.csv'
iris_data_preprocessor = DataPreprocessor(iris_file_path, separator=',')
iris_data = iris_data_preprocessor.load_data()
binarized_data = iris_data_preprocessor.binarize_data(iris_data)
iris_data_train, iris_data_test = iris_data_preprocessor.split_data(binarized_data)
iris_target_column = iris_data_train.columns[-1]

nb_classifier_log.fit(iris_data_train, iris_target_column)
iris_predictions = nb_classifier_log.predict(iris_data_test)

iris_result_df = iris_data_test.copy()
iris_result_df['Prediction'] = iris_predictions
print("---------------------------------------------------------------------------")
print("\nTest Data and Prediction results of the Naive Bayes classifier for iris (continuous data):")
print(iris_result_df.head(5))
iris_error_rate = nb_classifier_log.error_rate(iris_data_test, iris_predictions, iris_target_column)
if iris_error_rate is not None:
    print(f"\nNaive Bayes Classifier for iris (continuous data) error rate: {iris_error_rate:.2f}")
else:
    print("\nError rate cannot be calculated because the test set does not contain the target column.")
