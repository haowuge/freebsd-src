FreeBSD LoongArch Notes:
---------------
一些小技巧等

# 恢复默认目录树
git clean -f -d -x  
git reset --hard HEAD  

# 退回上一个提交
git reset --hard HEAD^
# 退回上三个提交
git reset --hard HEAD~3

# 打补丁
patch -p1 < ~/xxx.patch

