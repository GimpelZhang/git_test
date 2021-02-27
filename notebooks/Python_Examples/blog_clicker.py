# -*- coding: UTF-8 -*-
"""

自动化页面点击demo

python2

"""
# 注意：urllib2只在Python2中有：
import urllib2
import time
import re
import random

totalurl = {}
unusedurl = {}

#生成ProxyHandler对象
def get_proxy():
    return urllib2.ProxyHandler({'http': "localhost:8580"})

#生成指向代理的url_opener
def get_proxy_http_opener(proxy):
    return urllib2.build_opener(proxy)

#获取指定URL指向的网页，调用了前两个函数
def get_file_content(url):
    opener = get_proxy_http_opener(get_proxy())
    content = opener.open(url).read()
    opener.close()
    #为方便正则匹配，将其中的换行符消掉
    return content.replace('\r', '').replace('\n', '')

#根据网页HTML代码抽取网页标题：：没用到
def extract_title(content):
    titleelem = re.findall('"new"}\'>.*<\/a>', content)[0]
    return re.match('"new"}\'>(.*)<\/a>', titleelem).group(1).strip()

#根据网页HTML代码抽取所有<a>标签中的URL
def extract_url(content):
    urllist = []

    aelems = re.findall('<a href=".*?<\/a>', content)
    for aelem in aelems:
        splits = aelem.split(' ')
        if len(splits) != 1:
            aelem = splits[1]
        # print aelem
        matches = re.match('href="(.*)"', aelem)
        # print matches
        if matches is not None:
            url = matches.group(1)
            # print url
            if re.match('https:更详细的检索', url) is not None:
                urllist.append(url)
    return urllist

#获取字符串格式的时间
def get_localtime():
    return time.strftime("%H:%M:%S", time.localtime())

#主函数
def begin_access(starturl):
    
    totalurl[starturl] = True
    unusedurl[starturl] = True
    print 'seq\ttime\ttitle\turl'
    
    i = 0
    while i < 100 and unusedurl:
        nexturl = unusedurl.keys()[0]
        print nexturl
        del unusedurl[nexturl]
        content = get_file_content(nexturl)

        #print content
        
        #title = extract_title(content)
        title = "article:: "

        urllist = extract_url(content)
        #print "url:::"
        # print urllist
        
        for url in urllist:
            if not totalurl.has_key(url):
                totalurl[url] = True
                unusedurl[url] = True
        
        print '%d\t%s\t%s\t%s' %(i, get_localtime(), title, nexturl)
        
        i = i + 1
        time.sleep(random.uniform(1.1,4.3))
    totalurl.clear()
    unusedurl.clear()
    

#调用主函数
startpg = ['网址1','网址2']
for i in range(150):
    # 循环
    print "循环::::::::     %d\t" %(i)
    for bloglist in startpg:
        print 'blogs in:  '+bloglist
        begin_access(bloglist)
    time.sleep(random.uniform(11.4,19.5))
