import numpy as np

# Takes a matrix and row reduces it into row echelon form
# Expects a numpy array of floats
def ref(A):

    print('Starting Reduction')
    print(A)
    A_solved = np.copy(A)
    n_rows = np.shape(A)[0]
    n_cols = np.shape(A)[1]
    current_row = 0
    p = []
    # First create the list or rows, p
    for i in range(n_rows):
        p.append(i)

    # Now reduce the matrix
    for current_col in range(n_cols):
        # Find the pivot element
        max = [0, 0]
        for j in range(current_row, n_rows):
            # Find the largest element
            if abs(A[p[j]][current_col]) > abs(max[0]):
                max = [A[p[j]][current_col], j]
        # Swap row p so the largest element is the pivot
        if max[0] != 0:
            temp = p[current_row]
            p[current_row] = p[max[1]]
            p[max[1]] = temp
            pivot = max[0]
            flag = 1
        else:
            flag = 0
        # Skip the column if every element is zero
        current_row += flag

        if flag == 1:
            # Print the matrix to the terminal for debugging
            for n_i in range(n_rows):
                A_solved[n_i] = np.copy(A[p[n_i]])
            print('\nRows Arranged properly')
            print('Pivot is ' + str(pivot))
            print('Currently operating on row: ' + str(current_row + 1) + ' and column: ' + str(current_col + 1))
            print(A_solved)

            # Calculate I_row for each row
            for j in range(current_row, n_rows):
                i_row = A[p[j]][current_col] / pivot
                # Compute the new element values
                for k in range(current_col, n_cols):
                    A[p[j]][k] = np.round(A[p[j]][k] - i_row * A[p[current_row - 1]][k], 8)

            # Print the matrix to the terminal for debugging
            for n_i in range(n_rows):
                A_solved[n_i] = np.copy(A[p[n_i]])
            print('\nRows Reduced')
            print(A_solved)

    # Re-order the rows then return
    for current_col in range(n_rows):
        A_solved[current_col] = np.copy(A[p[current_col]])

        # General Pseudocode
        # Find pivot element (Largest)
        # Switch the corresponding entries in p
        # for i+1 to n in p (rows), set I_row = row_head / pivot
        # for i+1 to n (columns), elem = elem - I_row * pivot_row_elem

    return A_solved

#A = np.array(np.mat('1 2 3; 5 6 7; 9 10 11'), subok=True)
#A = np.array([[0, 0, 0, 0, 0], [8, 5, 4, 7, 3], [1, 2, 3, 4, 5], [9, 2, 6, 4, 3]], dtype=float)
#A = ref(A)
#A = np.array([[0, 4, 6, 3, 7], [8, 5, 4, 7, 3], [1, 2, 3, 4, 5], [9, 2, 6, 4, 3]], dtype=float)
#A = ref(A)
#A = np.array([[8, 2, 3, 4, 5], [5, 7, 0, 0, 4], [0, 9, 0, 0, 1], [0, 0, 0, 0, 7]], dtype=float)
#A = ref(A)
A = np.array([[2, 1, 5, 4, 2], [4, 2, 6, 10, 6], [8, 4, 12, 12, 10], [4, 2, 10, 8, 4]], dtype=float)
A = ref(A)