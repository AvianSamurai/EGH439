import numpy as np
import matplotlib.pyplot as plt 

def expand_section(matrix, x, y):
    """
    Expands the section containing the number 2 in a matrix in all directions.

    Args:
        matrix (list of list): The input matrix.
        x (int): The x-coordinate of the initial position of the section.
        y (int): The y-coordinate of the initial position of the section.
        expansion (int): Number of times to expand the section in all directions.

    Returns:
        list of list: The modified matrix.
    """
    # Check if the given coordinates are within the matrix bounds
    if x < 0 or y < 0 or x >= len(matrix) or y >= len(matrix[0]):
        return matrix

    # Check if the initial position contains 2
    if matrix[x][y] != 2:
        return matrix

    # Define the directions to expand: up, down, left, right
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

    # Iterate over each direction
    for dx, dy in directions:
        # Expand in the current direction
        for _ in range(1):
            new_x = x + dx
            new_y = y + dy

            # Check if the new coordinates are within the matrix bounds
            if 0 <= new_x < len(matrix) and 0 <= new_y < len(matrix[0]):
                # Check if the new position contains 0, if yes, expand
                if matrix[new_x][new_y] == 0:
                    matrix[new_x][new_y] = 3
                else:
                    break  # Stop expanding if encounter a non-zero value
            else:
                break  # Stop expanding if reach the matrix boundary

    return matrix

def expand_section2(matrix, x, y, counter):
    """
    Expands the section containing the number COUNTER in a matrix in all directions.

    Args:
        matrix (list of list): The input matrix.
        x (int): The x-coordinate of the initial position of the section.
        y (int): The y-coordinate of the initial position of the section.
        expansion (int): Number of times to expand the section in all directions.

    Returns:
        list of list: The modified matrix.
    """
    # Check if the given coordinates are within the matrix bounds
    if x < 0 or y < 0 or x >= len(matrix) or y >= len(matrix[0]):
        return matrix

    # Check if the initial position contains 2
    if matrix[x][y] != counter:
        return matrix

    # Define the directions to expand: up, down, left, right
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    # Iterate over each direction
    for dx, dy in directions:
        # Expand in the current direction
        for _ in range(1):
            new_x = x + dx
            new_y = y + dy

            # Check if the new coordinates are within the matrix bounds
            if 0 <= new_x < len(matrix) and 0 <= new_y < len(matrix[0]):
                # Check if the new position contains 0, if yes, expand
                if matrix[new_x][new_y] == 0:
                    matrix[new_x][new_y] = (counter + 1)
                else:
                    break  # Stop expanding if encounter a non-zero value
            else:
                break  # Stop expanding if reach the matrix boundary

    directions2 = [(1, 1), (-1, -1), (1, -1), (-1, 1)]
    # Iterate over each direction
    for dx, dy in directions2:
        # Expand in the current direction
        for _ in range(1):
            new_x = x + dx
            new_y = y + dy

            # Check if the new coordinates are within the matrix bounds
            if 0 <= new_x < len(matrix) and 0 <= new_y < len(matrix[0]):
                # Check if the new position contains 0, if yes, expand
                if matrix[new_x][new_y] == 0:
                    matrix[new_x][new_y] = (counter + 1.5)
                else:
                    break  # Stop expanding if encounter a non-zero value
            else:
                break  # Stop expanding if reach the matrix boundary

    return matrix

def return_section(matrix, x, y, counter2):
    """
    Expands the section containing the number COUNTER in a matrix in all directions.

    Args:
        matrix (list of list): The input matrix.
        x (int): The x-coordinate of the initial position of the section.
        y (int): The y-coordinate of the initial position of the section.
        expansion (int): Number of times to expand the section in all directions.

    Returns:
        list of list: The modified matrix.
    """
    # Check if the given coordinates are within the matrix bounds
    if x < 0 or y < 0 or x >= len(matrix) or y >= len(matrix[0]):
        return matrix

    # Define the directions to expand: up, down, left, right
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    smallest = np.inf
    smallest_space = [0,0]

    # Iterate over each direction
    for dx, dy in directions:
        # Expand in the current direction
        for _ in range(1):
            new_x = x + dx
            new_y = y + dy

            # Check if the new coordinates are within the matrix bounds
            if 0 <= new_x < len(matrix) and 0 <= new_y < len(matrix[0]):
                # Check if the new position contains 0, if yes, expand
                if matrix[new_x][new_y] < smallest and matrix[new_x][new_y] != np.pi:
                    smallest = matrix[new_x][new_y]
                    smallest_space[0] = new_x
                    smallest_space[1] = new_y
                else:
                    break  # Stop expanding if encounter a non-zero value
            else:
                break  # Stop expanding if reach the matrix boundary

    x = smallest_space[0]
    y = smallest_space[1]
    counter2 = smallest
    return matrix, x, y, counter2

def replace_3_with_2(matrix):
    """
    Replaces all occurrences of 3 with 2 in a given matrix.

    Args:
        matrix (list of list): The input matrix.

    Returns:
        list of list: The modified matrix.
    """
    # Iterate over each element in the matrix
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            # Check if the current element is 3
            if matrix[i][j] == 3:
                # Replace 3 with 2
                matrix[i][j] = 2

    return matrix

def replace_2_with_inf(matrix):
    """
    Replaces all occurrences of 2 with inf in a given matrix.

    Args:
        matrix (list of list): The input matrix.

    Returns:
        list of list: The modified matrix.
    """
    # Iterate over each element in the matrix
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            # Check if the current element is 2
            if matrix[i][j] == 2:
                # Replace 2 with inf
                matrix[i][j] = np.inf

    return matrix

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FINISH_IMPORT = [50,50]
EXPANTION = 7

OBSTICAL_IMPORT = [[-10,5,-10,5],[8,14,25,30],[40,45,-45,-40],[0,0,0,0]]

def Run(CART_IMPORT):

    MAP = np.ndarray(shape=(101,101))
    MAP[:,:] = 0

    print("think")

    for i in range(0, 4):
        for n in range((OBSTICAL_IMPORT[i][2]), (OBSTICAL_IMPORT[i][3])):
        #for j in range(10, 18):  
            for j in range((OBSTICAL_IMPORT[i][0]), (OBSTICAL_IMPORT[i][1])):
            #for n in range(10, 12):
                MAP[(n + 50) ,(j + 50)] = 2
                #print(MAP[(n + 50),(j + 50)])
                #print(j)
                #print(n)
                #print(" ")

    print("thunk")
    MAP[(CART_IMPORT[0]+50),(CART_IMPORT[1]+50)] = 1
    MAP[(FINISH_IMPORT[0]+50),(FINISH_IMPORT[1]+50)] = np.pi
    matrix = MAP
    # # Example usage:
    # matrix = [
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 1, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 2, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 0, np.pi]
    # ]

    for i in range(0, EXPANTION):
        for x in range(0, len(matrix)):
            for y in range(0, len(matrix[0])):
                expanded_matrix = expand_section(matrix, x, y)
        matrix = replace_3_with_2(expanded_matrix)

    matrix = replace_2_with_inf(expanded_matrix)

    print("thank")

    counter = 1
    while np.all(matrix) != True:
        for x in range(0, len(matrix)):
            for y in range(0, len(matrix[0])):
                expanded_matrix_solution = expand_section2(matrix, x, y, counter)
        counter = counter + 0.5
        #print(counter)

    print("thonk")

    counter2 = counter
    prev = 100
    x = (FINISH_IMPORT[0]+50)
    y = (FINISH_IMPORT[1]+50)
    arr2x = np.array([(FINISH_IMPORT[0]+50)])
    arr2y = np.array([(FINISH_IMPORT[1]+50)])

    while prev != 1:
        return_matrix_solution = return_section(matrix, x, y, counter2)
        matrix = return_matrix_solution[0]
        x = return_matrix_solution[1]
        y = return_matrix_solution[2]
        counter2 = return_matrix_solution[3]
        arr3x = np.array([y])
        arr3y = np.array([x])
        arr2x = np.concatenate((arr3x, arr2x), axis=0)
        arr2y = np.concatenate((arr3y, arr2y), axis=0)
        prev = matrix[x][y]
        matrix[x][y] = np.pi
        #print(y,x)

    print("done")

    combinedx = arr2x
    combinedy = arr2y
    print(combinedx)
    print(combinedy)

    plt.plot(combinedx, (-1 * combinedy))
    for i in range(0, 4):
        plt.plot([(OBSTICAL_IMPORT[i][0] + 50), (OBSTICAL_IMPORT[i][0] + 50), (OBSTICAL_IMPORT[i][1] + 50), (OBSTICAL_IMPORT[i][1] + 50), (OBSTICAL_IMPORT[i][0] + 50)], [(-1 * (OBSTICAL_IMPORT[i][2] + 50)), (-1 *(OBSTICAL_IMPORT[i][3] + 50)), (-1 * (OBSTICAL_IMPORT[i][3] + 50)), (-1 * (OBSTICAL_IMPORT[i][2] + 50)), (-1 * (OBSTICAL_IMPORT[i][2] + 50))])

    plt.show()

    return (combinedx, combinedy)

    # print(np.all(matrix))
    # for row in matrix:
    #     print(row)