#include "types.h"
#include "stat.h"
#include "user.h"

int main(void) {
    int count = getProcCount();
    printf(1, "successful : %d \n", count);
    exit();

}