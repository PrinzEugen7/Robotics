int countLines(char* fname)
{
    FILE *fp;
    int i = 0;
    double a,b,c;
    // エラー処理
    if((fp=fopen(fname,"r"))==NULL)
    {
        printf("FILE not open\n");
        exit(1);
    }
    // データ読み込み
    while(feof(fp) == 0)
    {
        fscanf(fp, "%lf%lf%lf", &a, &b, &c);
        i+=1;
    }
    fclose(fp);
    printf("%d\n", i);
    return i+1;
}
 
 
class Txt{
private:
public:
    Txt(){
    }
     
    double *x, *y, *z;
    int lines;
     
    void loadTxt(char *fname)
    {
        FILE *fp;
        int i = 0;
        // 行数分だけの要素をもつ配列の生成
        x = (double *)malloc(sizeof(double) * countLines(fname));
        y = (double *)malloc(sizeof(double) * countLines(fname));
        z = (double *)malloc(sizeof(double) * countLines(fname));   
        // エラー処理
        if((fp=fopen(fname,"r"))==NULL)
        {
            printf("FILE not open\n");
            exit(1);
        }
        // データ読み込み
        while(feof(fp) == 0)
        {
            fscanf(fp, "%lf%lf%lf", &x[i], &y[i], &z[i]);
            i = i + 1;
        }
        fclose(fp);
        lines = i+1;
    }
    // テキストファイルに書き込み
    void saveTxt(char *fname, double rx, double ry, double rz)
    {
        // ローカル変数の定義
        FILE *fp;
        // エラー処理
        if((fp=fopen(fname,"a"))==NULL)
        {
            printf("FILE not open\n");
            exit(1);
        }
        // データ書き込み
        fprintf(fp, "%0.2f\t%0.2f\t%0.2f\n", rx, ry, rz);
        fclose(fp);
    }
};
