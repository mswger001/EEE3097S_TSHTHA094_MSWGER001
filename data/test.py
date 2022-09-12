import parser_logger

path_to_data = "./f4/"
path_to_output = "./out_F2/"
verbose = 0

parser = parser_logger.Parser_logger(path=path_to_data, path_output=path_to_output)
parser.process_folder(verbose=verbose)