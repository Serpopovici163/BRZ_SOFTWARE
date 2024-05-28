#Chat GPTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
import pandas as pd
import re

# Read the Excel file
excel_file = 'CAN_ICD.xlsx'  # Update with your file path
df = pd.read_excel(excel_file)

# Extract relevant columns
# Assuming the columns are: CAN ID, (ignored), Variable Name, Comment
can_id_col = df.columns[0]
variable_name_col = df.columns[2]
comment_col = df.columns[3]

# Open the output C header file
with open('CAN_ICD.h', 'w') as file:
    file.write('// This file is generated from an Excel file containing CAN IDs and variable names\n\n')
    
    for index, row in df.iterrows():
        can_id = row[can_id_col]
        variable_name = row[variable_name_col]
        comment = row[comment_col]
        
        # Check if the variable name is not empty
        if pd.notna(variable_name) and variable_name.strip():
            # Replace any non-alphanumeric characters with underscores
            variable_name = re.sub(r'\W', '_', variable_name)
            # Write the #define statement with a comment
            file.write(f'#define CAN_ID_{variable_name} {can_id} // {comment}\n')

print('CAN_ICD.h file has been generated successfully.')