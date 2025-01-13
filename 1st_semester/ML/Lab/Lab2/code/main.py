import numpy as np
import matplotlib.pyplot as plt
from data_loader import DataLoader
from linear_regression import LinearRegressionModel
from evaluation import Evaluator



np.random.seed(42)


def main():
    """
    Main function to execute linear regression tasks on Turkish stock exchange
    and MT Cars datasets, including model training, testing, and evaluation.
    """
    # Task 1: Get data
    turkish_file = '../data/turkish-se-SP500vsMSCI.csv'
    mtcars_file = '../data/mtcarsdata-4features.csv'
    data_loader = DataLoader(turkish_file, mtcars_file)
    turkish_data, mtcars_data = data_loader.load_data()


    # Task 2: Fit a linear regression model
    print("----------------------------------------")
    print("Task 2: Fit a linear regression model")

    # Task 2_1: One-dimensional problem without intercept on the Turkish stock exchange data
    print("----------------------------------------")
    print("Task 2_1: One-dimensional problem without intercept on the Turkish stock exchange data")
    X_turkish = turkish_data.iloc[:, 0].values.reshape(-1, 1)
    y_turkish = turkish_data.iloc[:, 1].values
    model_no_intercept = LinearRegressionModel(fit_intercept=False)
    evaluator_no_intercept = Evaluator(model_no_intercept)
    
    print("Turkish Stock Exchange Data (No Intercept)")
    theta, train_mse, test_mse = evaluator_no_intercept.test_model(X_turkish, y_turkish)
    plt.scatter(X_turkish, y_turkish, color="blue", label="Original data")
    plt.plot(X_turkish, model_no_intercept.predict(X_turkish), color="red", label="Regression Line")
    plt.title("Turkish Stock Exchange Data - No Intercept")
    plt.xlabel("S&P500")
    plt.ylabel("MSCI")
    plt.legend()
    plt.show()

    # Tsk2_2: Compare graphically the solution obtained on different random subsets (10%) of the whole data set
    print("----------------------------------------")
    print("Task 2_2: Compare graphically the solution obtained on different random subsets (10%) of the whole data set")
    print("Turkish Stock Exchange Data (Random Subset Comparison - No Intercept)")
    evaluator_no_intercept.fit_and_plot_random_subsets(X_turkish, y_turkish)
    
    # Task2_3: One-dimensional problem with intercept on the Motor Trends car data, using columns mpg and weight
    print("----------------------------------------")
    print("Task 2_3: One-dimensional problem with intercept on the Motor Trends car data, using columns mpg and weight")
    model_with_intercept = LinearRegressionModel(fit_intercept=True)
    evaluator_with_intercept = Evaluator(model_with_intercept)
    X_mtcars = mtcars_data[['weight']].values
    y_mtcars = mtcars_data['mpg'].values
    print("MT Cars Data")
    evaluator_with_intercept.test_model(X_mtcars, y_mtcars)
    
    plt.scatter(X_mtcars, y_mtcars, color='blue')
    plt.plot(X_mtcars, model_with_intercept.predict(X_mtcars), color='red')
    plt.title("MT Cars: Weight vs MPG")
    plt.xlabel("Weight")
    plt.ylabel("MPG")
    plt.show()

    # Task2_4: Multi-dimensional problem on the complete MTcars data, using all four columns (predict mpg with the other three columns)
    print("----------------------------------------")
    print("Task 2_4: Multi-dimensional problem on the complete MTcars data, using all four columns (predict mpg with the other three columns)")
    X_multivariate = mtcars_data[['disp', 'hp', 'weight']].values
    y_multivariate = mtcars_data['mpg'].values
    print("MT Cars Data - Multi-dimensional regression")
    evaluator_with_intercept.test_model(X_multivariate, y_multivariate)


    # Task 3: Test regression model
    print("----------------------------------------")
    print("Task 3: Test regression model")
    
    # Task 3_1: Turkish stock exchange data (No Intercept)
    print("----------------------------------------")
    print("Task 3_1: Turkish stock exchange data (No Intercept)")
    X_turkish = turkish_data.iloc[:, 0].values.reshape(-1, 1)
    y_turkish = turkish_data.iloc[:, 1].values
    model_no_intercept = LinearRegressionModel(fit_intercept=False)
    evaluator_no_intercept = Evaluator(model_no_intercept)
    print("Turkish Stock Exchange Data (No Intercept) - 5% Test Data")
    evaluator_no_intercept.evaluate_on_random_splits(X_turkish, y_turkish)

    # Task 3_3: MT Cars data, mpg and weight with intercept
    print("----------------------------------------")
    print("Task 3_3: MT Cars data, mpg and weight with intercept")
    X_mtcars = mtcars_data[['weight']].values
    y_mtcars = mtcars_data['mpg'].values
    model_with_intercept = LinearRegressionModel(fit_intercept=True)
    evaluator_with_intercept = Evaluator(model_with_intercept)
    print("MT Cars Data (Weight vs MPG, With Intercept) - 5% Test Data")
    evaluator_with_intercept.evaluate_on_random_splits(X_mtcars, y_mtcars)
    
    # Task 3_4: MT Cars multi-dimensional data with intercept
    print("----------------------------------------")
    print("Task 3_4: MT Cars multi-dimensional data with intercept")
    X_multivariate = mtcars_data[['disp', 'hp', 'weight']].values
    y_multivariate = mtcars_data['mpg'].values
    print("MT Cars Data (Multi-dimensional) - 5% Test Data")
    evaluator_with_intercept.evaluate_on_random_splits(X_multivariate, y_multivariate)



if __name__ == "__main__":
    main()
