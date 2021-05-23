
#include <iostream>

extern "C" {
	#include "lua.h"
	#include "lualib.h"
	#include "lauxlib.h"
}




int main ()
{

  lua_State* L;

  L = lua_open();

  //load Lua base libraries 
  luaL_openlibs(L);

   //initialize Lua 
  luaL_dofile(L,"/home/sudhir/Dropbox/DDPG/lua_files/DeepPolicyGradientLearner.lua");
  //luaL_dofile(L,"/home/sudhir/Dropbox/DDPG/lua_files/test_1_lua.lua");
  
  //double z;

  /* push functions and arguments */
  //lua_getglobal(L, "fact");  /* function to be called */
  //lua_pushnumber(L, 5);   /* push 1st argument */
  //lua_pushnumber(L, 4);   /* push 2nd argument */
  
  lua_getglobal(L, "initialize");
  lua_call(L, 0, 1);
  //z = lua_tonumber(L, -1);
  //lua_pop(L, 1);  /* pop returned value */
  lua_close(L);

  //std::cout << z << std::endl;
  return 0;

}



