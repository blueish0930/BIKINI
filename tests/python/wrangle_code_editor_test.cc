#include "UI_code_editor.hh"
#include <cassert>
#include <iostream>
using namespace blender::ui;
int main() {
 const std::string a="if(x){if(y){a();b();}else{c();}}else{d();}";
 const std::string expected="if(x){\n  if(y){\n    a();\n    b();\n  } else{\n    c();\n  }\n} else{\n  d();\n}\n";
 assert(code_editor_format(a)==expected);
 const std::string b="for(int i=0;i<4;i++){string s=\"{;}\"; // if(){;}\n/* a;{\n} */if(i){f();}}";
 const auto fmt=code_editor_format(b);
 assert(fmt.find("for(int i=0;i<4;i++){")!=std::string::npos);
 assert(fmt.find("\"{;}\"")!=std::string::npos);
 assert(fmt.find("// if(){;}\n")!=std::string::npos);
 assert(fmt.find("/* a;{\n} */")!=std::string::npos);
 for(const auto &s: {a,b,std::string(""),std::string("do{a();}while(x);"),std::string("vector p[]={1,2,3};"),std::string("#define F(x) x;\\\n x\nf();")}) {
  const auto formatted=code_editor_format(s);
  assert(code_editor_format(formatted)==formatted);
 }
 assert(code_editor_normalize_paste("\tfoo();\r\n\t\tbar();")=="  foo();\n    bar();");
 assert(code_editor_normalize_paste("string s=\"\t\";\n\tf();")=="string s=\"\t\";\n  f();");
 assert(code_editor_normalize_paste("// user's comment\n\tf();")=="// user's comment\n  f();");
 CodeSearchState search;
 strcpy(search.find,"foo");search.update("foo x foo\nfoo");
 assert(search.matches.size()==3 && search.current==0);
 search.step(-1);assert(search.current==2);
 search.step(1);assert(search.current==0);
 search.step(1);assert(search.current==1);
 search.update("foo");assert(search.current==0 && search.matches.size()==1);
 search.update("");assert(search.current==-1 && search.matches.empty());
 search.step(-1);assert(search.current==-1);
 strcpy(search.find,"");search.update("foo");assert(search.matches.empty());
 std::cout<<"WRANGLE_EDITOR_KERNEL_PASS\n";
}
