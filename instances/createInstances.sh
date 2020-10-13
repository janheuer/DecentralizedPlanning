# subfolder of original/ have to exist in graph/ and traffic/

# create graph instances
for file in ./original/*.lp ./original/*/*.lp
do
    newfile="./graph/${file#"./original/"}"
    clingo --out-atomf='%s.' $file createGraph.lp | head -n 5 | tail -n 1 > $newfile
    sed -i 's/ /&\n/g' $newfile
done

# create traffic instances
for file in ./original/11x7/*.lp ./original/16x10/*.lp
do
    newfile="./traffic/${file#"./original/"}"
    clingo --out-atomf='%s.' $file createTraffic.lp | head -n 5 | tail -n 1 > $newfile
    sed -i 's/ /&\n/g' $newfile
done
