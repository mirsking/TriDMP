function Save2TXT(savePath,input_mattrix)
    fid=fopen(savePath,'wt');%д���ļ�·��
    matrix=input_mattrix;                        %input_matrixΪ���������
    [m,n]=size(matrix);
    for i=1:1:m
        for j=1:1:n
            if j==n
                fprintf(fid,'%g\n',matrix(i,j));
            else
                fprintf(fid,'%g\t',matrix(i,j));
            end
        end
    end
    fclose(fid);
end