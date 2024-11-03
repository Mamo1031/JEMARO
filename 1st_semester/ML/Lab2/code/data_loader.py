import pandas as pd



class DataLoader:
    """
    A class to load data from files for analysis.

    Attributes:
        turkish_file (str): Path to the Turkish stock exchange data file.
        mtcars_file (str): Path to the MT Cars data file.
    """
    def __init__(self, turkish_file, mtcars_file):
        """
        Initializes the DataLoader with file paths.

        Parameters:
            turkish_file (str): Path to the Turkish stock exchange data file.
            mtcars_file (str): Path to the MT Cars data file.
        """
        self.turkish_file = turkish_file
        self.mtcars_file = mtcars_file


    def load_data(self):
        """
        Loads the Turkish stock exchange and MT Cars datasets.

        Returns:
            tuple: Two pandas DataFrames, one for Turkish stock data and one for MT Cars data.
        """
        turkish_data = pd.read_csv(self.turkish_file)
        mtcars_data = pd.read_csv(self.mtcars_file)
        mtcars_data.columns = mtcars_data.columns.str.strip()  # Strip whitespace from column names
        
        return turkish_data, mtcars_data
