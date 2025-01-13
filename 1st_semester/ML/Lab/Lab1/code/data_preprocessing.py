import pandas as pd



class DataPreprocessor:
    def __init__(self, file_path, train_rate=0.72, separator=','):
        self.file_path = file_path
        self.train_rate = train_rate
        self.separator = separator


    def load_data(self):
        """
        Load the data from the CSV file.

        Returns:
            data (DataFrame): Loaded data
        """
        data = pd.read_csv(self.file_path, sep=self.separator, header=0)

        return data
    

    def split_data(self, data):
        """
        Split the loaded data into training and test sets.

        Parameters:
            data (DataFrame): The data to split.

        Returns:
            data_train (DataFrame): Training data
            data_test (DataFrame): Test data
        """
        train_size = int(len(data) * self.train_rate)
        data_train = data.sample(n=train_size, random_state=42)
        data_test = data.drop(data_train.index)

        return data_train, data_test
    
    
    def binarize_data(self, data):
        """
        Binarize the continuous features in the data by replacing values
        above the mean with 1 and those below or equal with 0.

        Parameters:
            data (DataFrame): The data to binarize.

        Returns:
            binarized_data (DataFrame): Binarized data
        """
        data.columns = ['sepal_length', 'sepal_width', 'petal_length', 'petal_width', 'target']

        # Calculate the mean for each feature and binarize the data
        means = data[['sepal_length', 'sepal_width', 'petal_length', 'petal_width']].mean()
        for feature in means.index:
            data[feature] = (data[feature] > means[feature]).astype(int)
        
        return data
