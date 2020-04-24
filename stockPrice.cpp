#include <iostream>
#include <string>
#include <vector>

using namespace std;

string stock;
vector<int> prices;
vector<int> result;

vector<int> solution(vector<int> prices) {
    vector<int> answer(prices.size(), 0);

    for (int i = 0; i < prices.size(); i++){
        for (int j = i+1; j < prices.size(); j++){
      
            if (prices[i] <= prices[j]){
                answer[i] = answer[i] + 1;
            }
            else if(prices[i] > prices[j]){
                answer[i]++;
                //i++;
                break;
            }
        }
    }
    answer[prices.size()-1] = 0;

    return answer;
}

int main(){
    cin >> stock;

    stock.erase(0, 1);
    stock.erase(stock.length()-1, 1);

    for (int i = 0; i < stock.length(); i++){
        if (stock.at(i) == ','){
            stock.erase(i, 1);
        }
        if (stock.at(i) == ' '){
            stock.erase(i, 1);
        }
    }

    for (int j = 0 ; j < stock.length(); j++){
        int n = stock[j] - 48;
        prices.push_back(n);
    }

    // for (int i = 0 ; i < prices.size(); i++){
    //     cout << prices[i] << " ";
    // }
    // cout << endl;

    result = solution(prices);

    cout << "[";

    for (int k = 0; k < result.size(); k++){
        cout << result[k];
        if (k != result.size()-1){
            cout << ",";
        }
    }

    cout << "]" << endl;
}

