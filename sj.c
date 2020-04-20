#include <stdio.h>

int digitReturn(int integer){
    int digit = 0;
    while(1){
        integer = integer / 10;
        digit ++;

        if (integer == 0) break;
    }

    return digit;
}

int main(void){
    
    while(1){
        int i = 0;
        int integer = 0;
        int digit = 0;
        int d = 0;

        // integer input 받아옴
        printf("Input an intenger (-1 to quit): \n");
        scanf("%d", &integer);

        // intger == -1이면 종료 
        if (integer == -1){
            printf("Bye!\n");
            break;
        }

        // digit 물어봄
        printf("Which digit do you want? \n");
        scanf("%d", &d);

        // integer 입력값의 digit 구하기
        digit = digitReturn(integer);

        //temp에 integer 임시 저장
        int temp = integer;
        for (int i = 0; i < d-1; i++){
            temp /= 10;
        }
        
        temp %= 10;

        printf("%d-th digit of %d is %d\n", d, integer, temp);


       
    }
}