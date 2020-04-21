#include <iostream>

using namespace std;

long long H, W;
int gcb;

void gcd(int a, int b){
    while (b != 0){
        long r = (long)a % b;
        a = b;
        b = r;
    }
    gcb = a;
}

long long solution(int w, int h){

    long long answer = 1;
    long long all = (long)w*h;

    gcd(w, h);

    return answer = all - (w + h - gcb);
}

int main(){
    cin >> W >> H;

    cout << solution(W, H);
}