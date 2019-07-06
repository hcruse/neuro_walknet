%module CommunicationInterface
%include "stl.i" 
%include "std_vector.i"
%include <boost_shared_ptr.i>

%{
#   define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS 
%}
 
namespace std {
   %template(StringVector) vector<std::string>;
};

%shared_ptr(BfbClient);

%warnfilter(401) PyObjectConverter;
%inline %{
#include <boost/variant.hpp>
class PyObjectConverter: public boost::static_visitor<PyObject*>{
	public:
		PyObject* operator()(const double input) const {
			return PyFloat_FromDouble(input);
		}
		PyObject* operator()(const long input) const {
			return PyInt_FromLong(input);
		}
		PyObject* operator()(const std::string& input) const {
			return PyString_FromString(input.c_str());
		}
};
%}

namespace AttributeUtilities {
	/* Convert from C++ --> Python */
	%typemap (out)(variableTypeList) {
		//PyObject* tempList=PyList_New(0);
		$result=PyList_New(0);
		for(auto it=$1.begin(); it!=$1.end(); ++it){
			PyObject* item=boost::apply_visitor(PyObjectConverter(), *it);
			PyList_Append($result,item);
		};
		
		//return tempList;
	};
	
	/* Convert from Python --> C++ */
	%typemap (in)(variableTypeList) {
		// Make sure that a list is passed to the next part of this method.
		PyObject *list;
		if(PyList_Check($input)){
			list=$input;
		}else if(PyTuple_Check($input)){
			list=PyList_New(PyTuple_Size($input));
			for(signed long int index=0;index<PyTuple_Size(list);index++){
				PyList_SetItem(list, index, PyTuple_GetItem($input, index));
			};
		}else if(PyFloat_Check($input) || PyLong_Check($input) || PyUnicode_Check($input)){
			list=PyList_New(0);
			PyList_Append(list, $input);
		}else{
			PyErr_SetString(PyExc_TypeError,"Expected a list/tuple/long/float/string, but got something else.");
			return NULL;
		};
		$1_type result;
		// After this point, a list is expected.
		result.reserve(PyList_Size(list));
		for (signed long int index = 0; index < PyList_Size(list); index++) {
			PyObject *item = PySequence_GetItem(list,index);
			if(PyFloat_Check(item)){
				result.push_back(PyFloat_AsDouble(item));
			}else if(PyLong_Check(item)){
				result.push_back(PyLong_AsLong(item));
			}else if(PyUnicode_Check(item)){
				PyObject* tempBytes=PyUnicode_AsUTF8String(item);
				std::string tempString=PyBytes_AsString(tempBytes); 
				result.push_back(tempString);
				Py_DECREF(tempBytes); 
			} else {
				PyErr_SetString(PyExc_ValueError,"Expected a long/float/string, but got something else.");
				return NULL;
			};
		};
		$1=result;
		Py_DECREF(list); 
	};
	
	%typemap (typecheck,precedence=200) variableTypeList{
		if(PyList_Check($input)){
			$1=1;
		}else if(PyTuple_Check($input)){
			$1=1;
		};
	};
}



/*
%extend BfbClient{
	%pythoncode{
	def __getattr__(self, attributeName):
		temp=GetValue(attributeName)
		if len(temp)==1:
			temp=temp[0]
		return temp
		
	def __setattr__(self, attributeName, values):
		typeOfValues=type(values)
		if typeOfValue in (int, float, str):
			values=[values]
		elif typeOfValue is list:
			pass
		elif typeOfValue is tuple:
			values=list(values)
		else:
			raise Exception('The passed type is neither int, float or string nor list or tuple')
			
		if typeOfValue is list and all([(type(v) in (int, float, string)) for v in values]):
			pass
		else:
			raise Exception('The passed list/tuple does not contain only int, float or string')

		SetValue(attributeName, values)
	}
};
*/		
		
%{
/* Includes the header in the wrapper code */
 #include "CommunicationInterface.hpp" 
 #include "BfbClient.hpp" 
 #include "Attribute.hpp" 
%}

/* Parse the header file to generate wrappers */
%include "CommunicationInterface.hpp"
%include "BfbClient.hpp"
%include "Attribute.hpp" 



%pythoncode{
def __getattr__bypass(self,name):
	if name!='this' and name!='thisown':
		if name=='errorState':
			return  _CommunicationInterface.BfbClient_GetErrorState(self)
		tempResult= _CommunicationInterface.BfbClient_GetValue(self, name)
		if len(tempResult)==1:
			tempResult=tempResult[0]
		return tempResult
	return _swig_getattr(self,BfbClient,name)

BfbClient.__getattr__=__getattr__bypass

def __setattr__bypass(self,name, value):
	if name!='this' and name!='thisown':
		typeOfValue=type(value)
		if typeOfValue in (int, float, str):
			value=[value]
		elif typeOfValue is list:
			pass
		elif typeOfValue is tuple:
			value=list(values)
		else:
			raise Exception('The passed type is neither int, float or string nor list or tuple')
		if all([(type(v) in (int, float, str)) for v in value]):
			pass
		else:
			raise Exception('The passed list/tuple does not contain only int, float or string')

		return _CommunicationInterface.BfbClient_SetValue(self, name, value)
	return _swig_setattr(self,BfbClient,name, value)

BfbClient.__setattr__=__setattr__bypass
}