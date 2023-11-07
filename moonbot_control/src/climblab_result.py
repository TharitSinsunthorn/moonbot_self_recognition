import csv

csv_file_path = '231107_053258_data.csv'

with open(csv_file_path, 'r') as file:
	csv_reader = csv.reader(file)
	data_list = []

	for row in csv_reader:
		data_list.append(row)

# for i in range(12):
# for j in range(2):

# 	print(data_list[j][111:])
seq = []
for j in range(1):
    # print(data_list[j][111:])
    float_generator = [item for item in data_list[j][114:117]]
    seq.append(float_generator)


print(seq)