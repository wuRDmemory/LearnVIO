#include "../../include/util/utils.h"

void printMatrix(string save_path, const Matrix<double, Dynamic, Dynamic, RowMajor>& matrix, int rows, int cols) {
    FILE* fp = fopen(save_path.c_str(), "w");
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fprintf(fp, "%lf,", matrix(i, j));
        }
        fprintf(fp, "\n");
    }
    fflush(fp);
    fclose(fp);
}
