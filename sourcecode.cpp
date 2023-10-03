#include <graphics.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <math.h>
#include <fstream>

#pragma comment( lib, "MSIMG32.LIB")

#define PI 3.1415926

struct universal_datatype {
	bool is_bool;
	bool is_string;
	bool is_number;

	bool if_bool_storage;
	std::string if_string_storage;
	float if_number_storage;
};

struct camera {
	int centralpositionx;
	int centralpositiony;
	int width;
	int height;
	int ONscreenpositionx;
	int ONscreenpositiony;
	int ONscreenwidth;
	int ONscreenheight;
};

bool isNum(std::string str)
{
    std::stringstream sin(str);
    double d;
    char c;
    if (!(sin >> d))
	{
	    return false;
	}
	if (sin >> c)
	{
      return false;
    }
	return true;
}


//碰撞体积类完成！
class Object {
public:

	int screenheight;
	int screenwidth;
	std::string objname = "newobj";
	bool objishard = true;

	//物体是否锁定运动
	bool islocked = false;
	bool isfrozen = false;

	//综合检测物体的碰撞
	bool objtopdetected = false;
	bool objbottomdetected = false;
	bool objleftdetected = false;
	bool objrightdetected = false;

	//物体设置
	float Rectwidth = 0, Rectheight = 0;
	float x = 0, y = 0;
	float lastx = 0, lasty = 0;

	std::vector<float> topxfriction;
	std::vector<float> bottomxfriction;
	std::vector<float> leftyfriction;
	std::vector<float> rightyfriction;
	float xfrictionmax = 1;
	float yfrictionmax = 1;
	float objairfriction = 1;

	//物理模拟
	float realx, realy;
	float vx = 0, vy = 0, ax = 0, ay = 0;
	bool isyfrict = false;
	bool isxfrict = false;

	//碰撞体积(绝对坐标)
	std::vector<std::string> rectname;
	std::vector<int>rectlefttopX;
	std::vector<int>rectlefttopY;
	std::vector<int>rectrightdownX;
	std::vector<int>rectrightdownY;

	//单一检测某碰撞块是否被物体碰撞
	std::vector<bool>recttopcollision;
	std::vector<bool>rectdowncollision;
	std::vector<bool>rectleftcollision;
	std::vector<bool>rectrightcollision;

	//通过触碰上下两边产生的X方向摩擦
	std::vector<bool>tXfrictPlus;
	std::vector<bool>bXfrictPlus;
	//通过触碰左右两边产生的Y方向摩擦
	std::vector<bool>lYfrictPlus;
	std::vector<bool>rYfrictPlus;

	//开启矩形的碰撞检测
	std::vector<bool>topconsole;
	std::vector<bool>downconsole;
	std::vector<bool>leftconsole;
	std::vector<bool>rightconsole;

	//矩形是否为刚体
	std::vector<bool> ishard;

	//增加碰撞体积
	void addRidgedbody(std::string name, int ltx, int lty, int rdx, int rdy) {
		this->rectname.push_back(name);
		this->rectlefttopX.push_back(ltx);
		this->rectlefttopY.push_back(lty);
		this->rectrightdownX.push_back(rdx);
		this->rectrightdownY.push_back(rdy);
		this->recttopcollision.push_back(false);
		this->rectdowncollision.push_back(false);
		this->rectleftcollision.push_back(false);
		this->rectrightcollision.push_back(false);
		this->topconsole.push_back(true);
		this->downconsole.push_back(true);
		this->leftconsole.push_back(true);
		this->rightconsole.push_back(true);
		this->tXfrictPlus.push_back(false);
		this->bXfrictPlus.push_back(false);
		this->lYfrictPlus.push_back(false);
		this->rYfrictPlus.push_back(false);
		this->topxfriction.push_back(1);
		this->bottomxfriction.push_back(1);
		this->leftyfriction.push_back(1);
		this->rightyfriction.push_back(1);
		this->ishard.push_back(true);
	}
	void scaleRigidbody(std::string name, int leftupx, int leftupy, int rightdownx, int rightdowny) {
		for (unsigned int i = 0; i < this->rectname.size(); i++)
		{
			if (name == this->rectname[i])
			{
				this->rectlefttopX[i] = leftupx;
				this->rectlefttopY[i] = leftupy;
				this->rectrightdownX[i] = rightdownx;
				this->rectrightdownY[i] = rightdowny;
				break;
			}
		}
	}
	void giveRidgedbodyName(std::string oldname, std::string newname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++)
		{
			if (oldname == this->rectname[i])
			{
				this->rectname[i] = newname;
				break;
			}
		}
	}
	//给物体一个力
	void giveforce(float forceX, float forceY) {
		if (!this->islocked) {
			ax += forceX;
			ay += forceY;
		}
	}
	//更新
	void update() {
		if (!isfrozen) {
			this->objtopdetected = false;
			this->objbottomdetected = false;
			this->objleftdetected = false;
			this->objrightdetected = false;

			float lastvx = this->vx;
			float lastvy = this->vy;
			float lastx = this->x;
			this->lastx = lastx;
			float lasty = this->y;
			this->lasty = lasty;
			this->vx += this->ax;
			this->vy += this->ay;

			//如果侦测到摩擦，就为定向动量乘上动摩擦因数
			if (this->isxfrict) {
				this->vx *= this->xfrictionmax;
			}
			if (this->isyfrict) {
				this->vy *= this->yfrictionmax;
			}

			this->vx *= this->objairfriction;
			this->vy *= this->objairfriction;
			this->xfrictionmax = 1;
			this->yfrictionmax = 1;

			//如果位移差值小于1，那么不再刷新
			if (this->vx - lastvx > -1 && this->vx - lastvx < 1) {
				this->isxfrict = false;
			}
			if (this->vy - lastvy > -1 && this->vy - lastvy < 1) {
				this->isyfrict = false;
			}

			this->x += this->vx;
			this->y += this->vy;

			this->giveforce(-this->ax, -this->ay);

			//如果小球碰到矩形，则将其对应方向上的速度归零
			for (unsigned int i = 0; (i < this->rectname.size()) && (this->rectname.size() != 0); i++) {
				//物体上边触墙
				if (this->downconsole[i]) {
					if (this->y + this->Rectheight / 2 >= this->rectrightdownY[i] && this->vy > 0 &&
						((this->x - this->Rectwidth / 2 >= this->rectlefttopX[i] && this->x - this->Rectwidth / 2 <= this->rectrightdownX[i] ||
							this->x + this->Rectwidth / 2 >= this->rectlefttopX[i] && this->x + this->Rectwidth / 2 <= this->rectrightdownX[i])
							||
							(this->x + this->Rectwidth / 2 >= this->rectrightdownX[i] && this->x - this->Rectwidth / 2 <= this->rectlefttopX[i])) &&
						lasty + this->Rectheight / 2 <= this->rectrightdownY[i]) {
						if (this->ishard[i] == true) {
							this->y = this->rectrightdownY[i] - this->Rectheight / 2;
							this->ay = 0;
							this->vy = 0;
						}
						this->bXfrictPlus[i] = true;
						this->rectdowncollision[i] = true;//矩形的下表面被obj碰到了！
						this->xfrictionmax *= this->bottomxfriction[i];
					}
					else {
						this->bXfrictPlus[i] = false;
						this->rectdowncollision[i] = false;
					}
				}

				//物体下边触墙
				if (this->topconsole[i]) {
					if (
						//底部y值小于等于矩形上部且物体运动趋势向下
						this->y - this->Rectheight / 2 <= this->rectlefttopY[i] && this->vy < 0 &&
						//物体左右确实是在矩形的左右范围内，判定为触碰
						((this->x - this->Rectwidth / 2 >= this->rectlefttopX[i] && this->x - this->Rectwidth / 2 <= this->rectrightdownX[i] ||
							this->x + this->Rectwidth / 2 >= this->rectlefttopX[i] && this->x + this->Rectwidth / 2 <= this->rectrightdownX[i])
							||
							(this->x + this->Rectwidth / 2 >= this->rectrightdownX[i] && this->x - this->Rectwidth / 2 <= this->rectlefttopX[i])) &&
						//判断上一个位置是否在该位置上面
						lasty - this->Rectheight / 2 >= this->rectlefttopY[i]) {
						if (this->ishard[i] == true) {
							this->y = this->rectlefttopY[i] + this->Rectheight / 2;
							this->ay = 0;
							this->vy = 0;
						}
						this->tXfrictPlus[i] = true;
						this->recttopcollision[i] = true;//矩形的上表面被obj碰到了！
						this->xfrictionmax *= this->topxfriction[i];
					}
					else {
						this->tXfrictPlus[i] = false;
						this->recttopcollision[i] = false;
					}
				}

				//物体左边触墙
				if (this->rightconsole[i]) {
					if (this->x - this->Rectwidth / 2 <= this->rectrightdownX[i] && this->vx < 0 &&
						((this->y + this->Rectheight / 2 <= this->rectlefttopY[i] && this->y + this->Rectheight / 2 >= this->rectrightdownY[i] ||
							this->y - this->Rectheight / 2 <= this->rectlefttopY[i] && this->y - this->Rectheight / 2 >= this->rectrightdownY[i])
							||
							(this->y + this->Rectheight / 2 >= this->rectlefttopY[i] && this->y - this->Rectheight / 2 <= this->rectrightdownY[i])) &&
						lastx - this->Rectwidth / 2 >= this->rectrightdownX[i]) {
						if (this->ishard[i] == true) {
							this->x = this->rectrightdownX[i] + this->Rectwidth / 2;
							this->ax = 0;
							this->vx = 0;
						}
						this->rYfrictPlus[i] = true;
						this->rectrightcollision[i] = true;//矩形的右表面被obj碰到了！
						this->yfrictionmax *= this->rightyfriction[i];
					}
					else {
						this->rYfrictPlus[i] = false;
						this->rectrightcollision[i] = false;
					}
				}

				//物体右边触墙
				if (this->leftconsole[i]) {
					if (this->x + this->Rectwidth / 2 >= this->rectlefttopX[i] && this->vx > 0 &&
						((this->y + this->Rectheight / 2 <= this->rectlefttopY[i] && this->y + this->Rectheight / 2 >= this->rectrightdownY[i] ||
							this->y - this->Rectheight / 2 <= this->rectlefttopY[i] && this->y - this->Rectheight / 2 >= this->rectrightdownY[i])
							||
							(this->y + this->Rectheight / 2 >= this->rectlefttopY[i] && this->y - this->Rectheight / 2 <= this->rectrightdownY[i])) &&
						lastx + this->Rectwidth / 2 <= this->rectlefttopX[i]) {
						if (this->ishard[i] == true) {
							this->x = this->rectlefttopX[i] - this->Rectwidth / 2;
							this->ax = 0;
							this->vx = 0;
						}
						this->lYfrictPlus[i] = true;
						this->rectleftcollision[i] = true;//矩形的左表面被obj碰到了！
						this->yfrictionmax *= this->leftyfriction[i];
					}
					else {
						this->lYfrictPlus[i] = false;
						this->rectleftcollision[i] = false;
					}
				}

				if (this->rectdowncollision[i] == true) {
					this->objtopdetected = true;
				}
				if (this->recttopcollision[i] == true) {
					this->objbottomdetected = true;
				}
				if (this->rectleftcollision[i] == true) {
					this->objrightdetected = true;
				}
				if (this->rectrightcollision[i] == true) {
					this->objleftdetected = true;
				}
			}

			this->isxfrict = false;
			this->isyfrict = false;
			for (unsigned int i = 0; (i < this->rectname.size()) && (this->rectname.size() != 0); i++) {
				if (this->tXfrictPlus[i] == true || this->bXfrictPlus[i] == true) {
					this->isxfrict = true;
				}
				if (this->lYfrictPlus[i] == true || this->rYfrictPlus[i] == true) {
					this->isyfrict = true;
				}
			}
			//下面这个更新的玩意一定一定在最后出场！
			this->realx = this->x;
			this->realy = this->screenheight - this->y;
		}
	}

	//初始化obj
	void initobj(std::string name, int x, int y, int width, int height) {
		this->x = x;
		this->y = y;
		this->Rectwidth = width;
		this->Rectheight = height;
		this->objname = name;
	}

	//物体运动锁定相关操作
	void lockobj() {
		this->islocked = true;
	}
	void unlockobj() {
		this->islocked = false;
	}
	void freezeobj() {
		this->isfrozen = true;
	}
	void unfreezeobj() {
		this->isfrozen = false;
	}

	//如果obj对于设定矩形的上/下/左/右端产生碰撞
	bool iftopCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				return this->recttopcollision[i];
			}
		}
		return false;
	}
	bool ifdownCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				return this->rectdowncollision[i];
			}
		}
		return false;
	}
	bool ifleftCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				return this->rectleftcollision[i];
			}
		}
		return false;
	}
	bool ifrightCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				return this->rectrightcollision[i];
			}
		}
		return false;
	}

	//如果obj的上下左右方向上产生了碰撞
	bool isobjtopCRASH() {
		return this->objtopdetected;
	}
	bool isobjbottomCRASH() {
		return this->objbottomdetected;
	}
	bool isobjleftCRASH() {
		return this->objleftdetected;
	}
	bool isobjrightCRASH() {
		return this->objrightdetected;
	}

	//禁止对设定矩形的上下左右检测
	void bantopCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->topconsole[i] = false;
				break;
			}
		}
	}
	void bandownCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->downconsole[i] = false;
				break;
			}
		}
	}
	void banleftCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->leftconsole[i] = false;
				break;
			}
		}
	}
	void banrightCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->rightconsole[i] = false;
				break;
			}
		}
	}

	//允许对设定矩形的上下左右检测
	void unbantopCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->topconsole[i] = true;
				break;
			}
		}
	}
	void unbandownCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->downconsole[i] = true;
				break;
			}
		}
	}
	void unbanleftCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->leftconsole[i] = true;
				break;
			}
		}
	}
	void unbanrightCRASH(std::string RECTname) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == RECTname) {
				this->rightconsole[i] = true;
				break;
			}
		}
	}

	//摩擦系数设定
	void settopxfriction(std::string name, float friction) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == name) {
				this->topxfriction[i] = friction;
				break;
			}
		}
	}
	void setbottomxfriction(std::string name, float friction) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == name) {
				this->bottomxfriction[i] = friction;
				break;
			}
		}
	}
	void setleftyfriction(std::string name, float friction) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == name) {
				this->leftyfriction[i] = friction;
				break;
			}
		}
	}
	void setrightyfriction(std::string name, float friction) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == name) {
				this->rightyfriction[i] = friction;
				break;
			}
		}
	}

	//设定碰撞体积是否为刚体
	void sethardness(std::string name, bool tf) {
		for (unsigned int i = 0; i < this->rectname.size(); i++) {
			if (this->rectname[i] == name) {
				this->ishard[i] = tf;
				break;
			}
		}
	}

	//获取obj移动趋势
	int getobjxmove() {
		return this->x - this->lastx;
	}
	int getobjymove() {
		return this->y - this->lasty;
	}
};

class System {
public:
	std::string system_status = "standby";

	std::map<std::string, bool> booldictionary;
	std::map<std::string, std::string> stringdictionary;
	std::map<std::string, float> numdictionary;

	camera cam;

	std::vector<Object> allobjects;

	//添加物体函数，没问题了已经
	void addObject(std::string name, int x, int y, int width, int height, int screenheight, int screenwidth) {
		Object newobj;
		newobj.initobj(name, x, y, width, height);
		newobj.screenheight = screenheight;
		newobj.screenwidth = screenwidth;
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			newobj.addRidgedbody(this->allobjects[i].objname,
				this->allobjects[i].x - this->allobjects[i].Rectwidth / 2,
				this->allobjects[i].y + this->allobjects[i].Rectheight / 2,
				this->allobjects[i].x + this->allobjects[i].Rectwidth / 2,
				this->allobjects[i].y - this->allobjects[i].Rectheight / 2);
			this->allobjects[i].addRidgedbody(name, x - width / 2, y + height / 2, x + width / 2, y - height / 2);
		}
		this->allobjects.push_back(newobj);
	}

	void setcamera(int midposx,int midposy,int width,int height, int Smidposx, int Smidposy, int Swidth, int Sheight) {
		this->cam.centralpositionx = midposx;
		this->cam.centralpositiony = midposy;
		this->cam.width = width;
		this->cam.height = height;
		this->cam.ONscreenpositionx = Smidposx;
		this->cam.ONscreenpositiony = Smidposy;
		this->cam.ONscreenwidth = Swidth;
		this->cam.ONscreenheight = Sheight;
	}

	//更新函数，没问题了已经
	void update() {//更新所有Object对象状态
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			this->allobjects[i].update();
		}
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			for (unsigned int j = 0; j < this->allobjects.size(); j++) {
				if (j != i) {
					this->allobjects[i].scaleRigidbody(this->allobjects[j].objname,
						this->allobjects[j].x - this->allobjects[j].Rectwidth / 2,
						this->allobjects[j].y + this->allobjects[j].Rectheight / 2,
						this->allobjects[j].x + this->allobjects[j].Rectwidth / 2,
						this->allobjects[j].y - this->allobjects[j].Rectheight / 2);
				}
			}
		}
	}

	//检测两个obj是否碰撞
	bool istouched(int ax, int ay, int awidth, int aheight, int bx, int by, int bwidth, int bheight) {//ab中心坐标与ab长宽
		if (ax + awidth / 2 >= bx - bwidth / 2 && ay - aheight / 2 <= by + bheight / 2
			&&
			ax - awidth / 2 <= bx + bwidth / 2 && ay + aheight / 2 >= by - bheight / 2) {
			return true;
		}
		else {
			return false;
		}
	}

	//给物体一个力
	void giveobjforce(std::string objname, int forcex, int forcey) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].giveforce(forcex, forcey);
			}
		}
	};//给予特定物体一个力

	//将物体移动到
	void moveto(std::string objname, int x, int y) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].x = x;
				this->allobjects[i].y = y;
				this->allobjects[i].ax = 0;
				this->allobjects[i].ay = 0;
				this->allobjects[i].vx = 0;
				this->allobjects[i].vy = 0;
			}
		}
	}

	void lockobject(std::string name) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == name) {
				this->allobjects[i].lockobj();
			}
		}
	}
	void unlockobject(std::string name) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == name) {
				this->allobjects[i].unlockobj();
			}
		}
	}
	void freezeobject(std::string name) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == name) {
				this->allobjects[i].freezeobj();
			}
		}
	}
	void unfreezeobject(std::string name) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == name) {
				this->allobjects[i].unfreezeobj();
			}
		}
	}

	//如果objname2的上下左右端碰到了objname
	bool iftopcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].iftopCRASH(objname2);
			}
		}
	}
	bool ifdowncrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].ifdownCRASH(objname2);
			}
		}
	}
	bool ifleftcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].ifleftCRASH(objname2);
			}
		}
	}
	bool ifrightcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].ifrightCRASH(objname2);
			}
		}
	}

	//如果objname的上下左右端被碰到了
	bool ifobjtopcrash(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].isobjtopCRASH();
			}
		}
	}
	bool ifobjbottomcrash(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].isobjbottomCRASH();
			}
		}
	}
	bool ifobjleftcrash(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].isobjleftCRASH();
			}
		}
	}
	bool ifobjrightcrash(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].isobjrightCRASH();
			}
		}
	}

	//禁止objname2的上下左右端对objname的移动产生影响
	void bantopcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].bantopCRASH(objname2);
				break;
			}
		}
	}
	void bandowncrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].bandownCRASH(objname2);
				break;
			}
		}
	}
	void banleftcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].banleftCRASH(objname2);
				break;
			}
		}
	}
	void banrightcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].banrightCRASH(objname2);
				break;
			}
		}
	}

	//允许objname2的上下左右端对objname的移动产生影响
	void unbantopcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].unbantopCRASH(objname2);
				break;
			}
		}
	}
	void unbandowncrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].unbandownCRASH(objname2);
				break;
			}
		}
	}
	void unbanleftcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].unbanleftCRASH(objname2);
				break;
			}
		}
	}
	void unbanrightcrash(std::string objname, std::string objname2) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].unbanrightCRASH(objname2);
				break;
			}
		}
	}

	//设置objname2的对于objname上下左右摩擦系数
	void settopxfriction(std::string objname, std::string objname2, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].settopxfriction(objname2, friction);
				break;
			}
		}
	}
	void setbottomxfriction(std::string objname, std::string objname2, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].setbottomxfriction(objname2, friction);
				break;
			}
		}
	}
	void setleftyfriction(std::string objname, std::string objname2, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].setleftyfriction(objname2, friction);
				break;
			}
		}
	}
	void setrightyfriction(std::string objname, std::string objname2, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].setrightyfriction(objname2, friction);
				break;
			}
		}
	}

	//设置objname的上下左右对于所有物体摩擦系数
	void settopxfrictionsingle(std::string objname, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				for (unsigned int j = 0; j < this->allobjects.size(); j++) {
					if (j != i) {
						this->settopxfriction(this->allobjects[j].objname, this->allobjects[i].objname, friction);
					}
				}
				break;
			}
		}
	}
	void setbottomxfrictionsingle(std::string objname, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				for (unsigned int j = 0; j < this->allobjects.size(); j++) {
					if (j != i) {
						this->setbottomxfriction(this->allobjects[j].objname, this->allobjects[i].objname, friction);
					}
				}
				break;
			}
		}
	}
	void setleftyfrictionsingle(std::string objname, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				for (unsigned int j = 0; j < this->allobjects.size(); j++) {
					if (j != i) {
						this->setleftyfriction(this->allobjects[j].objname, this->allobjects[i].objname, friction);
					}
				}
				break;
			}
		}
	}
	void setrightyfrictionsingle(std::string objname, float friction) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				for (unsigned int j = 0; j < this->allobjects.size(); j++) {
					if (j != i) {
						this->setrightyfriction(this->allobjects[j].objname, this->allobjects[i].objname, friction);
					}
				}
				break;
			}
		}
	}


	//设置objname2对于objname是否为刚体
	void sethardness(std::string objname, std::string objname2, bool tf) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				this->allobjects[i].sethardness(objname2, tf);
				break;
			}
		}
	}
	//设置objname是否为刚体
	void sethardness(std::string objname, bool tf) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				for (unsigned int j = 0; j < this->allobjects.size(); j++) {
					if (j != i) {
						this->sethardness(this->allobjects[j].objname, this->allobjects[i].objname, tf);
					}
				}
				break;
			}
		}
	}

	//获取物体横纵方向上的位移
	int getobjxmove(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].getobjxmove();
			}
		}
	}
	int getobjymove(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i].getobjymove();
			}
		}
	}

	//打印
	void drawrect() {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			rectangle(this->allobjects[i].realx - this->allobjects[i].Rectwidth / 2,
				this->allobjects[i].realy - this->allobjects[i].Rectheight / 2,
				this->allobjects[i].realx + this->allobjects[i].Rectwidth / 2,
				this->allobjects[i].realy + this->allobjects[i].Rectheight / 2);
		}
	}

	Object callbackobj(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return this->allobjects[i];
			}
		}
	}
	int callbackobjindex(std::string objname) {
		for (unsigned int i = 0; i < this->allobjects.size(); i++) {
			if (this->allobjects[i].objname == objname) {
				return i;
			}
		}
	}

	//TODO:设计两个序列，使得运行后程序循环判断读取的condition内容，根据是否触发再确定后面的result是否执行

	universal_datatype deepen(std::string function_name, std::string parameters) {
		int cntleft = 0;
		std::string buffer;
		std::vector<std::string> functionName;
		std::vector<std::string> functionParameter;
		for (int i = 0; i < parameters.length(); i++) {//将源码按逗号切割
			if (parameters[i] == *const_cast<char*>(" ")) {
				continue;
			}
			if (parameters[i] == *const_cast<char*>("("))
				cntleft++;
			if (parameters[i] == *const_cast<char*>(")"))
				cntleft--;
			if (cntleft == 0 && parameters[i] == *const_cast<char*>(",")) {
				functionName.push_back(buffer);
				buffer.clear();
			}
			else {
				buffer.push_back(parameters[i]);
			}
			if (i == parameters.length() - 1) {
				functionName.push_back(buffer);
			}
		}
		for (int i = 0; i < functionName.size(); i++) {//如果是函数，那么将functionName里的函数名和参数分开存储,如果不是函数，就直接存储在functionName里，不做改变
			bool is_function = false;
			for (int m = 0; m < functionName[i].size(); m++) {
				if (functionName[i][m] == *const_cast<char*>("(") || functionName[i][m] == *const_cast<char*>(")"))
					is_function = true;
			}
			if (is_function == false) {
				functionParameter.push_back("");
				continue;
			}
			std::string buffer = "";
			std::string parameterBuffer = "";
			for (int j = 0; j < functionName[i].length(); j++)//将函数名分离
				if (functionName[i][j] != *const_cast<char*>("("))
					buffer.push_back(functionName[i][j]);
				else {
					break;
				}
			for (int k = buffer.length() + 1; k < functionName[i].length() - 1; k++)//将分离出来的参数记录
				parameterBuffer.push_back(functionName[i][k]);
			functionName[i] = buffer;
			functionParameter.push_back(parameterBuffer);
			buffer.clear();
			parameterBuffer.clear();
		}

		//OK to here
		if (function_name == "STRUCT") {
			for (int i = 0; i < functionName.size();i++) {
				deepen(functionName[i], functionParameter[i]);
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "UNTIL") {
			for (; deepen(functionName[0], functionParameter[0]).if_bool_storage == false;) {
				for (int i = 1; i < functionName.size(); i++) {
					deepen(functionName[i], functionParameter[i]);
				}
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "openwindow") {
			if (deepen(functionName[0], functionParameter[0]).is_number == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false 
				) {
				std::cout << "openwindow的参数应当为number,number类型" << std::endl;
				system("pause");
			}
			initgraph(deepen(functionName[0], functionParameter[0]).if_number_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "cleardevice") {
			cleardevice();
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "beginbatchdraw") {
			BeginBatchDraw();
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "flushbatchdraw") {
			FlushBatchDraw();
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "drawrect") {
			drawrect();
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "sleep") {
			if (deepen(functionName[0], functionParameter[0]).is_number == false)
			{
				std::cout << "sleep的参数应当为number类型" << std::endl;
				system("pause");
			}
			Sleep(deepen(functionName[0], functionParameter[0]).if_number_storage);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "update") {
			this->update();
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "puttext") {
			if (deepen(functionName[0], functionParameter[0]).is_number == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false ||
				deepen(functionName[2], functionParameter[2]).is_string == false
				) {
				std::cout << "puttext的参数应当为number,number,string类型" << std::endl;
				system("pause");
			}
			int buffers[2] = { deepen(functionName[1], functionParameter[1]).if_number_storage,
				(int)deepen(functionName[0], functionParameter[0]).if_number_storage };
			std::string buffer = deepen(functionName[2], functionParameter[2]).if_string_storage;
			for (int i = 0; i < buffer.size(); i++) {
				outtextxy(buffers[1] + 10 * i,
					buffers[0],
					TCHAR(buffer.c_str()[i]));
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "IF") {
			if (deepen(functionName[0], functionParameter[0]).is_bool == false ) {
				std::cout << "IF的参数应当为bool,null类型" << std::endl;
				system("pause");
			}
			if (deepen(functionName[0], functionParameter[0]).if_bool_storage) {
				for (int i = 1; i < functionName.size();i++) {
					deepen(functionName[i], functionParameter[i]);
				}
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				true,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_window_nowoperating") {
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				GetHWnd() == GetForegroundWindow(),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_keyboard_pushed") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "if_keyboard_pushed的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				(bool)(GetAsyncKeyState(deepen(functionName[0], functionParameter[0]).if_string_storage[0]) & 0x8000),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "add_object") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false ||
				deepen(functionName[3], functionParameter[3]).is_number == false ||
				deepen(functionName[4], functionParameter[4]).is_number == false ||
				deepen(functionName[5], functionParameter[5]).is_number == false ||
				deepen(functionName[6], functionParameter[6]).is_number == false
				) {
				std::cout << "add_object的参数应当为string,number,number,number,number,number,number类型" << std::endl;
				system("pause");
			}
			this->addObject(deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage,
				deepen(functionName[3], functionParameter[3]).if_number_storage,
				deepen(functionName[4], functionParameter[4]).if_number_storage,
				deepen(functionName[5], functionParameter[5]).if_number_storage,
				deepen(functionName[6], functionParameter[6]).if_number_storage);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "is_touched") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "add_object的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				this->istouched(
					this->callbackobj(deepen(functionName[0], functionParameter[0]).if_string_storage).x,
					this->callbackobj(deepen(functionName[0], functionParameter[0]).if_string_storage).y,
					this->callbackobj(deepen(functionName[0], functionParameter[0]).if_string_storage).Rectwidth,
					this->callbackobj(deepen(functionName[0], functionParameter[0]).if_string_storage).Rectheight,
					this->callbackobj(deepen(functionName[1], functionParameter[1]).if_string_storage).x,
					this->callbackobj(deepen(functionName[1], functionParameter[1]).if_string_storage).y,
					this->callbackobj(deepen(functionName[1], functionParameter[1]).if_string_storage).Rectwidth,
					this->callbackobj(deepen(functionName[1], functionParameter[1]).if_string_storage).Rectheight
				),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "give_obj_force") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "give_obj_force的参数应当为string,number,number类型" << std::endl;
				system("pause");
			}
			this->giveobjforce(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0 
			};
			return returnal;
		}
		else if (function_name == "move_to") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "move_to的参数应当为string,number,number类型" << std::endl;
				system("pause");
			}
			this->moveto(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "lock_object") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "lock_object的参数应当为string类型" << std::endl;
				system("pause");
			}
			this->lockobject(
				deepen(functionName[0], functionParameter[0]).if_string_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "unlock_object") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "unlock_object的参数应当为string类型" << std::endl;
				system("pause");
			}
			this->unlockobject(
				deepen(functionName[0], functionParameter[0]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "freeze_object") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "freeze_object的参数应当为string类型" << std::endl;
				system("pause");
			}
			this->freezeobject(
				deepen(functionName[0], functionParameter[0]).if_string_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0 
			};
			return returnal;
		}
		else if (function_name == "unfreeze_object") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "unfreeze_object的参数应当为string类型" << std::endl;
				system("pause");
			}
			this->unfreezeobject(
				deepen(functionName[0], functionParameter[0]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0 
			};
			return returnal;
		}
		else if (function_name == "if_top_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "if_top_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,this->iftopcrash(
					deepen(functionName[0], functionParameter[0]).if_string_storage,
					deepen(functionName[1], functionParameter[1]).if_string_storage
				),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_down_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "if_down_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				this->ifdowncrash(
					deepen(functionName[0], functionParameter[0]).if_string_storage,
					deepen(functionName[1], functionParameter[1]).if_string_storage
				),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_left_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "if_left_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				this->ifleftcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
				)
				,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_right_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "if_right_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				this->ifrightcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
				)
				,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_object_top_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "if_object_top_crash的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				this->ifobjtopcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage
				)
				,
				"",
				0 };
			return returnal;
		}
		else if (function_name == "if_object_bottom_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "if_object_bottom_crash的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				this->ifobjbottomcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage
				)
				,"",0 };
			return returnal;
		}
		else if (function_name == "if_object_left_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "if_object_left_crash的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				this->ifobjleftcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage
				),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "if_object_right_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false
				) {
				std::cout << "if_object_right_crash的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				this->ifobjrightcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage),
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "ban_top_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "ban_top_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->bantopcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "ban_down_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "ban_down_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->bandowncrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "ban_left_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "ban_left_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->banleftcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "ban_right_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "ban_right_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->banrightcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "unban_top_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "unban_top_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->unbantopcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "unban_down_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "unban_down_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->unbandowncrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "unban_left_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "unban_left_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->unbanleftcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "unban_right_crash") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false
				) {
				std::cout << "unban_right_crash的参数应当为string,string类型" << std::endl;
				system("pause");
			}
			this->unbanrightcrash(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_objairfriction") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false
				) {
				std::cout << "set_objairfriction的参数应当为string,number类型" << std::endl;
				system("pause");
			}
			this->allobjects[this->callbackobjindex(deepen(functionName[0], functionParameter[0]).if_string_storage)].objairfriction = deepen(functionName[1], functionParameter[1]).if_number_storage;
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_top_xfriction") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "set_top_xfriction的参数应当为string,string,number类型" << std::endl;
				system("pause");
			}
			this->settopxfriction(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_bottom_xfriction") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "set_bottom_xfriction的参数应当为string,string,number类型" << std::endl;
				system("pause");
			}
			this->setbottomxfriction(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_left_yfriction") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "set_left_yfriction的参数应当为string,string,number类型" << std::endl;
				system("pause");
			}
			this->setleftyfriction(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_right_yfriction") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_string == false ||
				deepen(functionName[2], functionParameter[2]).is_number == false
				) {
				std::cout << "set_right_yfriction的参数应当为string,string,number类型" << std::endl;
				system("pause");
			}
			this->setrightyfriction(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_string_storage,
				deepen(functionName[2], functionParameter[2]).if_number_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0 
			};
			return returnal;
		}
		else if (function_name == "set_top_xfriction_single") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false 
				) {
				std::cout << "set_top_xfriction_single的参数应当为string,number类型" << std::endl;
				system("pause");
			}
			this->settopxfrictionsingle(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_bottom_xfriction_single") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false
				) {
				std::cout << "set_bottom_xfriction_single的参数应当为string,number类型" << std::endl;
				system("pause");
			}
			this->setbottomxfrictionsingle(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_left_yfriction_single") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false
				) {
				std::cout << "set_left_yfriction_number的参数应当为string,number类型" << std::endl;
				system("pause");
			}
			this->setleftyfrictionsingle(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage
			);
			universal_datatype returnal;
			returnal = {
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "set_right_yfriction_single") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false ||
				deepen(functionName[1], functionParameter[1]).is_number == false
				) {
				std::cout << "set_right_yfriction_single的参数应当为string,number类型" << std::endl;
				system("pause");
			}
			this->setrightyfrictionsingle(
				deepen(functionName[0], functionParameter[0]).if_string_storage,
				deepen(functionName[1], functionParameter[1]).if_number_storage
			);
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				false,
				false,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "ET") {
			if ((!deepen(functionName[0], functionParameter[0]).is_bool || !deepen(functionName[1], functionParameter[1]).is_bool) &&
				(!deepen(functionName[0], functionParameter[0]).is_string || !deepen(functionName[1], functionParameter[1]).is_string)) {
				std::cout << "EqualTo的参数应当为number或bool类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			if (deepen(functionName[0], functionParameter[0]).is_bool == true) {
				returnal = {
					true,
					false,
					false,
					deepen(functionName[0],functionParameter[0]).if_bool_storage == deepen(functionName[1],functionParameter[1]).if_bool_storage,
					"",
					0 
				};
			}
			else if (deepen(functionName[0], functionParameter[0]).is_number == true) {
				returnal = { true,false,false,deepen(functionName[0],functionParameter[0]).if_number_storage == deepen(functionName[1],functionParameter[1]).if_number_storage,"",0 };
			}
			return returnal;
		}
		else if (function_name == "NE") {
			if ((!deepen(functionName[0], functionParameter[0]).is_bool || !deepen(functionName[1], functionParameter[1]).is_bool) &&
				(!deepen(functionName[0], functionParameter[0]).is_string || !deepen(functionName[1], functionParameter[1]).is_string)) {
				std::cout << "NotEqual的参数应当为number或bool类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			if (deepen(functionName[0], functionParameter[0]).is_bool == true) {
				returnal = { 
					true,
					false,
					false,
					deepen(functionName[0],functionParameter[0]).if_bool_storage != deepen(functionName[1],functionParameter[1]).if_bool_storage,
					"",
					0
				};
			}
			else if (deepen(functionName[0], functionParameter[0]).is_number == true) {
				returnal = {
					true,
					false,
					false,
					deepen(functionName[0],functionParameter[0]).if_number_storage != deepen(functionName[1],functionParameter[1]).if_number_storage,
					"",
					0 
				};
			}
			return returnal;
		}
		else if (function_name == "LT") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "LessThan的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_number_storage < deepen(functionName[1],functionParameter[1]).if_number_storage,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "LE") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "LessEqual的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_number_storage <= deepen(functionName[1],functionParameter[1]).if_number_storage,
				"",
				0 };
			return returnal;
		}
		else if (function_name == "GT") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "GreaterThan的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_number_storage > deepen(functionName[1],functionParameter[1]).if_number_storage,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "GE") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "GreaterEqual的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_number_storage >= deepen(functionName[1],functionParameter[1]).if_number_storage,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "AND") {
			if (!deepen(functionName[0], functionParameter[0]).is_bool || !deepen(functionName[1], functionParameter[1]).is_bool) {
				std::cout << "AND的参数应当为bool类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_bool_storage && deepen(functionName[1],functionParameter[1]).if_bool_storage,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "OR") {
			if (!deepen(functionName[0], functionParameter[0]).is_bool || !deepen(functionName[1], functionParameter[1]).is_bool) {
				std::cout << "OR的参数应当为bool类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				true,
				false,
				false,
				deepen(functionName[0],functionParameter[0]).if_bool_storage || deepen(functionName[1],functionParameter[1]).if_bool_storage,
				"",
				0
			};
			return returnal;
		}
		else if (function_name == "NOT") {
			if (!deepen(functionName[0], functionParameter[0]).is_bool) {
				std::cout << "NOT的参数应当为bool类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				true,
				false,
				false,
				!deepen(functionName[0],functionParameter[0]).if_bool_storage,
				"",
				0
			};
			return returnal;
			}
		else if (function_name == "ADD") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "ADD的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				deepen(functionName[0],functionParameter[0]).if_number_storage + deepen(functionName[1],functionParameter[1]).if_number_storage 
			};
			return returnal;
		}
		else if (function_name == "SUB") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "SUB的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				true,
				false,
				"",
				deepen(functionName[0],functionParameter[0]).if_number_storage - deepen(functionName[1],functionParameter[1]).if_number_storage 
			};
			return returnal;
		}
		else if (function_name == "MUL") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "MUL的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				deepen(functionName[0],functionParameter[0]).if_number_storage * deepen(functionName[1],functionParameter[1]).if_number_storage 
			};
			return returnal;
		}
		else if (function_name == "DIV") {
			if (!deepen(functionName[0], functionParameter[0]).is_number || !deepen(functionName[1], functionParameter[1]).is_number) {
				std::cout << "DIV的参数应当为number类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				deepen(functionName[0],functionParameter[0]).if_number_storage / deepen(functionName[1],functionParameter[1]).if_number_storage 
			};
			return returnal;
		}
		else if (function_name == "pushback_displacement_x") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_displacement_x的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->getobjxmove(deepen(functionName[0],functionParameter[0]).if_string_storage)
			};
			return returnal;
		}
		else if (function_name == "pushback_displacement_y") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_displacement_y的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->getobjymove(deepen(functionName[0],functionParameter[0]).if_string_storage)
			};
			return returnal;
		}
		else if (function_name == "pushback_position_x") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_position_x的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).x
			};
			return returnal;
		}
		else if (function_name == "pushback_position_y") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_position_y的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).y
			};
			return returnal;
		}
		else if (function_name == "pushback_printed_x") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_printed_x的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).realx
			};
			return returnal;
			}
		else if (function_name == "pushback_printed_y") {
				if (deepen(functionName[0], functionParameter[0]).is_string == false) {
					std::cout << "pushback_printed_y的参数应当为string类型" << std::endl;
					system("pause");
				}
				universal_datatype returnal;
				returnal = {
					false,
					false,
					true,
					false,
					"",
					(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).realy
				};
				return returnal;
				}
		else if (function_name == "pushback_height") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_height的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).Rectheight 
			};
			return returnal;
		}
		else if (function_name == "pushback_width") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_width的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).Rectwidth
			};
			return returnal;
		}
		else if (function_name == "pushback_window_width") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_window_width的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = { 
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).screenwidth 
			};
			return returnal;
		}
		else if (function_name == "pushback_window_height") {
			if (deepen(functionName[0], functionParameter[0]).is_string == false) {
				std::cout << "pushback_window_height的参数应当为string类型" << std::endl;
				system("pause");
			}
			universal_datatype returnal;
			returnal = {
				false,
				false,
				true,
				false,
				"",
				(float)this->callbackobj(deepen(functionName[0],functionParameter[0]).if_string_storage).screenheight 
			};
			return returnal;
		}
		//函数在这里继续加
		else { //如果不是函数，那么就是变量
			if (function_name == "false") {
				universal_datatype returnal;
				returnal = {
					true,
					false,
					false,
					false,
					"",
					0
				};
				return returnal;
			}
			else if (function_name == "true") {
				universal_datatype returnal;
				returnal = { 
					true,
					false,
					false,
					true,
					"",
					0 
				};
				return returnal;
			}
			else if (isNum(function_name)) {
				universal_datatype returnal;
				returnal = { 
					false,
					false,
					true,
					false,
					"",
					(float)std::atof(function_name.c_str())
				};
				return returnal;
			}
			else if (function_name[0] == '"' && function_name[function_name.length() - 1] == '"') {
				universal_datatype returnal;
				returnal = { 
					false,
					true,
					false,
					false,
					function_name.substr(1,function_name.length() - 2),
					0 
				};
				return returnal;
			}
			else {
				if (function_name == "SYSTEM_STATUS") {//再不然，就是系统关键字调用
					universal_datatype returnal;
					returnal = { 
						false,
						true,
						false,
						false,
						this->system_status,
						0 
					};
					return returnal;
				}
			}
			//如果连变量都不是，那么就是未知对象
			//后期可以加一个报错列表，使得错误集中存储在列表中集中输出，防止一次只报错一个错误内容
			std::cout << "未定义的方法名：" << function_name << std::endl;
			system("pause");
		}
		//还有其他的判定的话在这里继续加，确保返回类型是returnal即可，如果是变量，那么也按照该变量类型写在returnal里作为返回值
		//请注意，如果是变量，那么它应当有且只有存在于functionName(function_name)中，而不是存储在functionParameter(parameters)中
		//TODO：如果都不符合，在这里报错（先疏通报错逻辑！！！！！！）
	}
};

IMAGE RotateImage_Alpha(IMAGE* pImg, double radian, COLORREF bkcolor = BLACK)
{
	radian = -radian;														// 由于 y 轴翻转，旋转角度需要变负
	float fSin = (float)sin(radian), fCos = (float)cos(radian);				// 存储三角函数值
	float fNSin = (float)sin(-radian), fNCos = (float)cos(-radian);
	int left = 0, top = 0, right = 0, bottom = 0;							// 旋转后图像顶点
	int w = pImg->getwidth(), h = pImg->getheight();
	DWORD* pBuf = GetImageBuffer(pImg);
	POINT points[4] = { {0, 0}, {w, 0}, {0, h}, {w, h} };					// 存储图像顶点
	for (int j = 0; j < 4; j++)												// 旋转图像顶点，搜索旋转后的图像边界
	{
		points[j] = {
			(int)(points[j].x * fCos - points[j].y * fSin),
			(int)(points[j].x * fSin + points[j].y * fCos)
		};
		if (points[j].x < points[left].x)	left = j;
		if (points[j].y > points[top].y)	top = j;
		if (points[j].x > points[right].x)	right = j;
		if (points[j].y < points[bottom].y)	bottom = j;
	}

	int nw = points[right].x - points[left].x;								// 旋转后的图像尺寸
	int nh = points[top].y - points[bottom].y;
	int nSize = nw * nh;
	int offset_x = points[left].x < 0 ? points[left].x : 0;					// 旋转后图像超出第一象限的位移（据此调整图像位置）
	int offset_y = points[bottom].y < 0 ? points[bottom].y : 0;

	IMAGE img(nw, nh);
	DWORD* pNewBuf = GetImageBuffer(&img);
	if (bkcolor != BLACK)													// 设置图像背景色
		for (int i = 0; i < nSize; i++)
			pNewBuf[i] = BGR(bkcolor);

	for (int i = offset_x, ni = 0; ni < nw; i++, ni++)						// i 用于映射原图像坐标，ni 用于定位旋转后图像坐标
	{
		for (int j = offset_y, nj = 0; nj < nh; j++, nj++)
		{
			int nx = (int)(i * fNCos - j * fNSin);							// 从旋转后的图像坐标向原图像坐标映射
			int ny = (int)(i * fNSin + j * fNCos);
			if (nx >= 0 && nx < w && ny >= 0 && ny < h)						// 若目标映射在原图像范围内，则拷贝色值
				pNewBuf[nj * nw + ni] = pBuf[ny * w + nx];
		}
	}

	return img;
}
void transparentimage(IMAGE* dstimg, int x, int y, IMAGE* srcimg)
{
	HDC dstDC = GetImageHDC(dstimg);
	HDC srcDC = GetImageHDC(srcimg);
	int w = srcimg->getwidth();
	int h = srcimg->getheight();
	BLENDFUNCTION bf = { AC_SRC_OVER, 0, 255, AC_SRC_ALPHA };
	AlphaBlend(dstDC, x, y, w, h, srcDC, 0, 0, w, h, bf);
}
//imgRotate = RotateImage_Alpha(&imgPng, f, 0x66AA0000);	// 图像旋转（设置了半透明的填充背景）
//transparentimage(NULL, 0, 0, &imgRotate);				// 透明贴图


float angle(float x1, float y1, float x2, float y2) {
	float angle_temp;
	float xx, yy;
	xx = x2 - x1;
	yy = y2 - y1;
	if (xx == 0.0)
		angle_temp = PI / 2.0;
	else
		angle_temp = atan(fabs(yy / xx));
	if ((xx < 0.0) && (yy >= 0.0))
		angle_temp = PI - angle_temp;
	else if ((xx < 0.0) && (yy < 0.0))
		angle_temp = PI + angle_temp;
	else if ((xx >= 0.0) && (yy < 0.0))
		angle_temp = PI * 2.0 - angle_temp;
	return (angle_temp);
}

int main() {
	//1.读取文件？（要有文件系统，确保执行准确）
	//2.初始化系统
	//3.开启循环
	//4.循环中不断抽取初始化后system里的conditions进行判断，并执行对应operations
	//5.直到有quit指令被触发
	//6.保存文件，退出系统
	//系统在线时要及时调整system_status确保响应状态准确
	System sys;
	std::string fullfile;

	std::ifstream file("example.txt");
	if (file.is_open())
	{
		std::string line;
		while (getline(file, line))
		{
			for (int i = 0; i < line.size();i++) {
				if (line[i] != ' ' && line[i] != '\n' && line[i] != '\t') {
					fullfile = fullfile + line[i];
				}
			}
		}
		sys.deepen("STRUCT", fullfile);
	}
	else
	{
		std::cout << "Unable to open Gamebus file" <<std::endl;
	}

	//sys.deepen("add_object", "\"example\",400,300,100,100,600,800");
	//sys.deepen("add_object","\"warrior\",500,400,100,100,600,800");

	//sys.deepen("set_objairfriction","\"example\",0.8");
	//initgraph(800,600);
	//while (true) {
	//	if (GetHWnd() == GetForegroundWindow()) {//探测操作窗口是否为当前窗口，后期在这里执行判定循环
	//		if (GetAsyncKeyState('D') & 0x8000) {//右移动
	//			sys.giveobjforce("example", 3, 0);
	//		}
	//		if (GetAsyncKeyState('A') & 0x8000) {//左移动
	//			sys.giveobjforce("example", -3, 0);
	//		}
	//		if (GetAsyncKeyState('W') & 0x8000) {//上
	//			sys.giveobjforce("example", 0, 3);
	//		}
	//		if (GetAsyncKeyState('S') & 0x8000) {//下
	//			sys.giveobjforce("example", 0, -3);
	//		}
	//		//GetCursorPos(&p);
	//	}

	//	BeginBatchDraw();//
	//	sys.update();
	//	outtextxy(sys.callbackobj("example").realx - sys.callbackobj("example").Rectwidth / 2, sys.callbackobj("example").realy - sys.callbackobj("example").Rectheight / 2,L"example");
	//	//成功测试字符串判定功能和内部输出功能，准备下一阶段实现命令行功能！！
	//	sys.drawrect();//
	//	FlushBatchDraw();//
	//	cleardevice();
	//	Sleep(10);//
	//}

	

	return 0;
}
