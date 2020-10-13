/*
    ©2010 AREXX Engineering
    http://www.arexx.com/
    ------------------------------------
    Design by Dominik S. Herwald
    contact: d.herwald@dsh-elektronik.de
*/


var menu_item_low='url(../images/main_menu_item_l.gif)';
var menu_item_high='url(../images/main_menu_item_h.gif)';
var sub_menu_item_low='url(../images/main_sub_menu_item_l.gif)';
var sub_menu_item_high='url(../images/main_sub_menu_item_h.gif)';
t_out=600;
var locked = new Array(32);
var locked2 = new Array(32);
var hover_enable = false;
var nav=navigator.userAgent.toLowerCase()
var version = parseInt(navigator.appVersion)
if(version >= 4)
    if((nav.indexOf('gecko')!=-1)||(nav.indexOf('msie')!=-1))
         hover_enable = true
if (nav.indexOf('opera')!=-1)
    hover_enable = false

for(var i=0;i<40;i++)
    lock('_item'+i);

function lock(item)
{
     locked[item]=true;
     if(locked2[item]!=null)
          window.clearTimeout(locked2[item]);
}

function unlock(item)
{
 //locked[item]=false;
}

function hl(item, link, sub)
{
    if(!sub)
    {
         document.getElementsByTagName('td')[item].style.background=menu_item_high;
         document.getElementsByTagName('a')[link].style.color='#000000';
    }
    else
    {
         document.getElementsByTagName('td')[item].style.background=sub_menu_item_high;
         document.getElementsByTagName('a')[link].style.color='#000000';
    }
}

function ll(item, link, sub)
{
    if(hover_enable)
    {
         locked2[item] = window.setTimeout('tll("'+item+'","'+link+'","'+sub+'")',t_out);
    }
    else
    {
        if(!sub)
        {
             document.getElementsByTagName('td')[item].style.background=menu_item_low;
             document.getElementsByTagName('a')[link].style.color='#ffffff';
        }
        else
        {
             document.getElementsByTagName('td')[item].style.background=sub_menu_item_low;
             document.getElementsByTagName('a')[link].style.color='#ffffff';
        }
    }
}

function tll(item,link,sub)
{
    if(sub!='true')
    {
         document.getElementsByTagName('td')[item].style.background=menu_item_low;
         document.getElementsByTagName('a')[link].style.color='#ffffff';
    }
    else
    {
         document.getElementsByTagName('td')[item].style.background=sub_menu_item_low;
         document.getElementsByTagName('a')[link].style.color='#ffffff';
    }
}
