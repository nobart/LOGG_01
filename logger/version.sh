#!/bin/bash
ROOT=$1
TEMP_FILENAME='svninfo.txt'
svn info $ROOT 2>/dev/null 1>svninfo.txt 
SVN_ROOT=`awk /'Repository Root:'/'{print $3}' $TEMP_FILENAME`
SVN_URL=`awk /'URL: svn.ssh'/'{print $2}' $TEMP_FILENAME`
SVN_REVISION_NUM=`awk /'Last Changed Rev:'/'{print $4}' $TEMP_FILENAME`
SVN_REVISION_SGN=0
rm -fr $TEMP_FILENAME
MODIFIED=`svn st -q $ROOT | wc -l `

######################################################################
# check if snapshot view modified 
if [ "$MODIFIED" != "0" ]
then
    REVISION_STR=`whoami`@`date -u +%Y%m%d%H%M%S`
    echo $REVISION_STR > crc.txt
    SVN_REVISION_SGN=0x`crc32 crc.txt`
    rm crc.txt
    echo  $REVISION_STR $SVN_REVISION_SGN
exit 0
fi
######################################################################

######################################################################
# check if this is a valid tag
TAGS=`expr match "$SVN_URL" "$SVN_ROOT/tags/FW"`
if [ "$TAGS" != "0" ] 
then
######################################################################
# tagged build
    REVISION_STR=${SVN_URL/"$SVN_ROOT/tags/FW"/} 
     #remove "firmware" text, replace %23 to #, replace / to _
    REVISION_STR=`echo $REVISION_STR | awk '{ gsub("firmware/", "", $0); gsub("branches/", "", $0); gsub("%23", "#", $0);  gsub("/", "_", $0); print}'`  
    echo $REVISION_STR > crc.txt
    SVN_REVISION_SGN=0x`crc32 crc.txt`
    rm crc.txt
    echo $REVISION_STR  $SVN_REVISION_SGN
exit 0
fi
######################################################################

######################################################################
# not a tagged build 
REVISION_STR=${SVN_URL/"$SVN_ROOT/"/}@$SVN_REVISION_NUM 
#remove "firmware" text, replace %23 to #, replace / to _
REVISION_STR=`echo $REVISION_STR | awk '{ gsub("firmware/", "", $0); gsub("branches/", "", $0); gsub("%23", "#", $0);  gsub("/", "_", $0); print}'`  
echo $REVISION_STR > crc.txt
SVN_REVISION_SGN=0x`crc32 crc.txt`
rm crc.txt
echo $REVISION_STR `echo $SVN_REVISION_NUM | awk '{printf("0x%08x", $1)}'` $SVN_REVISION_SGN
exit 0
######################################################################
