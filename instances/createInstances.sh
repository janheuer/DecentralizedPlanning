# subfolder of original/ have to exist in graph/ and traffic/
for file in ./original/*.lp ./original/*/*.lp
do
    newfile="./graph/${file#"./original/"}"
    clingo --out-atomf='%s.' original/instance.lp createGraph.lp | head -n 5 | tail -n 1 > $newfile
    sed -i 's/ /&\n/g' $newfile

    newfile="./traffic/${file#"./original/"}"
    clingo --out-atomf='%s.' original/instance.lp createTraffic.lp | head -n 5 | tail -n 1 > $newfile
    sed -i 's/ /&\n/g' $newfile
done
