import { PanelExtensionContext, RenderState} from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState } from "react";
import ReactDOM from "react-dom";
import type {Parameter, ParameterValue, SetSrvParam} from "parameter_types";

// MAIN BRANCH //

let node: string;
let paramNameList: string[];
let paramValList: ParameterValue[];


function ExamplePanel({ context }: { context: PanelExtensionContext }): JSX.Element {


  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [status, setStatus] = useState<string | undefined>();

  const [paramList, setParamList] = useState<Array<Parameter>>();
  const [srvParamList, setSrvParamList] = useState<Array<SetSrvParam>>();
  const [nodeList, setNodeList] = useState<string[]>();

  const [colorScheme, setColorScheme] = useState<string>();
  const [bgColor, setBgColor] = useState("#d6d6d6");
  const [saveButtonBgColor, setSaveButtonBgColor] = useState("#d6d6d6");
  const [loadButtonBgColor, setLoadButtonBgColor] = useState("#d6d6d6");

  const [paramsToYaml, setParamsToYaml] = useState<string>();

  useLayoutEffect( () => {

    context.onRender = (renderState: RenderState, done) => { 

      setRenderDone(() => done); 
      updateNodeList();

      //Manage some styling for light and dark theme
      setColorScheme(renderState.colorScheme);
      if(renderState.colorScheme == "light") {
        setBgColor("#d6d6d6");
        setSaveButtonBgColor("#d6d6d6");
        setLoadButtonBgColor("#d6d6d6");
      } else if (renderState.colorScheme == "dark") {
        setBgColor("#4d4d4d");
        setSaveButtonBgColor("#4d4d4d");
        setLoadButtonBgColor("#4d4d4d");
      }
    };

    //If new topics are found, context.onRender() will update the list of nodes
    context.watch("topics");

    //If colorScheme changes, context.onRender() will change styling to match new color scheme
    context.watch("colorScheme");

  }, []);

  // invoke the done callback once the render is complete
  useEffect(() => {  
    renderDone?.();
  }, [renderDone]);

  /**
   * converts string representation of a boolean to a boolean
   * @param stringValue "true" or "false"
   * @returns true or false
   */
  const stringToBoolean = (stringValue: string) => {
    switch(stringValue?.toLowerCase()?.trim()){
        case "true": return true;
        case "false": return false;
        default: return undefined;
    }
  }

  /**
   * determines if a string[] contains exlusively booleans
   * @param strArr string[] to check  
   * @returns true if strArr only contains booleans, false otherwise
   */
  const isBooleanArr = (strArr: string[]) => {
    let bool: boolean = true;
    strArr.forEach(element => {
      console.log(stringToBoolean(element));
      if(stringToBoolean(element) === undefined)
        bool = false;
    });

    return bool;
  }


  /**
   * Returns the string value of a paramter's value to be outputted on the screen
   * @param   param The parameter value that is converted to a string
   * @returns String representation of param
  */
  const getParameterValue = (param: ParameterValue) => {
    if(param === undefined) { return "undefined"; }
    switch(param.type) {
      case 1:  return param.bool_value.toString();
      case 2:  return param.integer_value.toString();
      case 3:  return param.double_value.toString();
      case 4:  return param.string_value;
      case 5:  return `[${param.byte_array_value.toString()}]`;
      case 6:  return `[${param.bool_array_value.toString()}]`;
      case 7:  return `[${param.integer_array_value.toString()}]`;
      case 8:  return `[${param.double_array_value.toString()}]`;
      case 9:  return `[${param.string_array_value.toString()}]`;
      default: return "error, invalid type...";
    }
  }

  /**
   * Updates the list of nodes when a new node appears
   */
  const updateNodeList = () => {
    setStatus("retreiving nodes...")
    context.callService?.("/rosapi/nodes", {})
    .then((_values: unknown) =>{ 
      setNodeList((_values as any).nodes as string[]);
      setStatus("nodes retreived");  
    })
    .catch((_error: Error) => { setStatus(_error.toString()); });
  }
  
  /**
   * Retrieves a list of all parameters for the current node and their values
   */
  const updateParamList = () =>{

    context.callService?.(node + "/list_parameters", {})
    .then((_value: unknown) => {
      paramNameList = (_value as any).result.names as string[];

      context.callService?.(node + "/get_parameters", {names: paramNameList})
      .then((_value: unknown) => {
        paramValList = (_value as any).values as ParameterValue[];

        let tempList:Array<Parameter> = [];
        for (let i = 0; i < paramNameList.length; i++) {
          tempList.push({name: paramNameList[i]!, value: paramValList[i]!});
        }
        if(tempList.length > 0) 
          setParamList(tempList);

        if(paramNameList !== undefined) {
          setSrvParamList(new Array(paramList?.length));
        }
      })
      .catch(() => {setStatus("error, failed to retreive parameter values")});
    })
    .catch(() => {setStatus("error, failed to retreive parameter list")});
  }

  /**
   * Sets new values to all parameters with an inputted new value
   * Calls updateParamList() to 'refresh' the screen and display the new parameter values
   */
  const setParam = () => {
    setStatus("setting parameters...");

    let tempList: SetSrvParam[] = srvParamList!;

    for(let i: number = 0; i < tempList.length; i++) {
      if(tempList[i] == null) {
        tempList.splice(i, 1);
        i = -1;
      }
    }

    setSrvParamList(tempList);
    context.callService?.(node + "/set_parameters", {parameters: srvParamList})
    .then(() => {
      updateParamList();
      setStatus("parameters set");
    })
    .catch((error: Error) => {
      updateParamList();
      setStatus("Error: " + JSON.stringify(error));
    });
  }


  let paramTypeList: string[] = ["boolean", "integer", "double", "string", "byte_array", "boolean_array", "integer_array", "double_array", "string_array"];

  /**
   * return the parameter type of the given parameter value
   * @param paramVal The given Parameter Value
   * @returns paramVal's parameter type
   */
  const getType = (paramVal: ParameterValue) => {
    if (paramVal === undefined)
      return "undefined";
    return paramTypeList[paramVal.type - 1];
  }

  /**
   * Update the list of Parameters with new values to be set 
   * @param val The new value to be set
   * @param name The name of the parameter that will be set to 'val'
   */
  const updateSrvParamList = (name: string, val: string) => {
    let idx: number = paramNameList?.indexOf(name)!;
    let tempList: SetSrvParam[] = srvParamList!;
    let tempValList: string[] = [];

    if(val === "") {
      const emptyP: SetSrvParam = {};
      tempList[idx] = emptyP;
    } else {
      let ssp: SetSrvParam = {};
      let valStrArr: string[] = [];
      switch (paramList![idx]?.value.type!) {
        case 1: 
          ssp = { name: name, value: { type: 1, bool_value: stringToBoolean(val) }}; 
          break;

        case 2: 
          ssp = { name: name, value: { type: 2, integer_value: +val }}; 
          break;

        case 3: 
          ssp = { name: name, value: { type: 3, double_value: +val }}; 
          break;

        case 4: 
          ssp = { name: name, value: { type: 4, string_value: val }}; 
          break;
 
        // TODO: Implement format for byte arrays
        case 5: 
          //ssp = { name: name, value: { type: 5, byte_array_value: val as unknown as number[] }}; 
          break;

        case 6:
          valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
          if(isBooleanArr(valStrArr)) {
            let valBoolArr: boolean[] = valStrArr.map((element) => {
              if(element == "true")
                return true;
              return false;
            });
            ssp = { name: name, value: { type: 6, bool_array_value: valBoolArr}};
          }
          break;

        case 7: 
        valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
          ssp = { name: name, value: { type: 7, integer_array_value: valStrArr.map(Number) }};
          break;

        case 8:
          valStrArr = val.replace(" ", "").replace("[", "").replace("]", "").split(",");
          ssp = { name: name, value: { type: 8, double_array_value: valStrArr.map(Number) }};
          break;

        case 9:
          val.replace(" ", "");
          if(val.charAt(0) == '[' && val.charAt(val.length - 1) == ']') 
            val = val.substring(1, val.length - 1);
          valStrArr = val.split(",");
          ssp = { name: name, value: { type: 9, string_array_value: valStrArr }}; 
          break;

        default: ssp = {}; break;
      }
      tempList[idx] = ssp;
      setSrvParamList(tempList)
    }

    paramValList.forEach(element => {
      tempValList.push(getParameterValue(element));
    });
  }

  /**
   * Gives the YAML format of a parameter and its value
   * @param pVal Parameter value in question
   * @returns pVal in YAML format
   */
  const getYamlValue = (pVal: ParameterValue) => {
    let value: string = "";
    switch (pVal.type!) {
      case 1: value = pVal.bool_value.toString(); break;
      case 2: value = pVal.integer_value.toString(); break;
      case 3: value = pVal.double_value.toString(); break;
      case 4: value = pVal.string_value; break;
      case 5:

        pVal.byte_array_value.forEach(val => {
          value = value.concat("\n\t\t- " + val.toString());
        });
        break;
      case 6: 
        pVal.bool_array_value.forEach(val => {
          value = value.concat("\n\t\t- " + val.toString());
        });
        break;
      case 7:
        pVal.integer_array_value.forEach(val => {
          value = value.concat("\n\t\t- " + val.toString());
        });
        break;
      case 8: 
        pVal.double_array_value.forEach(val => {
          value = value.concat("\n\t\t- " + val.toString());
        });
        break;
      case 9: 
        pVal.string_array_value.forEach(val => {
          value = value.concat("\n\t\t- " + val);
        });
        break;
      default:
    }
    return value;
  }

  /**
   * Writes current parameter list to a config file in YAML format
   */
  const saveParamsToFile = () => {
    let yaml: string = node + ":\n\t" + "ros__parameters:\n\t\t";
      (paramList ?? []).map((result) => (
        yaml = yaml.concat(result.name + ": " + getYamlValue(result.value) + "\n\t\t")
      ))
    setParamsToYaml(yaml);

  }

  
  /**
   * Creates a dropdown input box if param is a boolean, creates a text input box otherwise
   * @param   param The parameter that an input box is being created for
   * @returns A dropdown if param.value.type == 1, a textbox otherwise
   */
  const createInputBox = (param: Parameter) => {
    if(param.value.type == 1) {
      return(
        <select
        style={dropDownStyle}
        onChange={(event) => { updateSrvParamList(param.name, event.target.value) }}
        >
          <option selected hidden></option>
          <option>true</option>
          <option>false</option>
        </select>
      );
    }
    return(
      <input style={inputStyle} placeholder={getParameterValue(param.value)} onChange={(event) => { updateSrvParamList(param.name, event.target.value) }}/> 
    );
  }


  /**
   * loads parameter values from a YAML file and sets all new values
   * @param files the YAML file to be uploaded
   */
  const loadFile = (files: FileList | null) => { 
    if(files !== null) {
      files[0]?.text()
      .then((value: string) => {      
        value = value.replaceAll(/[^\S\r\n]/gi, "");
        value = value.replace(node + ":\n", "");
        value = value.replace("ros__parameters:\n", "");

        let params: string[] = value.split("\n");
        for(let i = 0; i < params.length; i++) {

          if(params[i]!.charAt(0) != '-' && params[i]!.charAt(params[i]!.length - 1) != ':') {
            let temp: string[]= params[i]!.split(":");
            updateSrvParamList(temp[0]!, temp[1]!);

          } else if(params[i]!.charAt(params[i]!.length - 1) == ':') {
            let tempName: string = params[i]!.replace(":", "").trim();
            let tempVal: string = "";

            while(i + 1 < params.length && params[++i]!.charAt(0) == '-') {
              tempVal = tempVal.concat(params[i]!.replace("-", "").trim() + ",");
            }

            i--;
            tempVal = tempVal.substring(0, tempVal.length-1);
            updateSrvParamList(tempName, tempVal);
          }
        }
        setParam();
      })
      .catch((error: Error) => {
        console.log(error)
      });
    }
  }

  ///////////////////////////////////////////////////////////////////
  //////////////////////// PANEL LAYOUT /////////////////////////////
  ///////////////////////////////////////////////////////////////////


  //////////////////////// CSS STYLING //////////////////////////////

  let setButtonStyle = {};
  let saveButtonStyle = {};
  let loadButtonStyle = {};
  let textAreaStyle = {};
  let dropDownStyle = {};
  let inputStyle = {};

  if(colorScheme == "light") {

    setButtonStyle = {

      fontSize: "1rem",
      backgroundColor: bgColor,
      border: bgColor + " solid",
      margin: "36px 12px 36px 0px",
      padding: "8px",
      borderRadius: "4px",
      color: "#333333",
      fontWeight: "500",

    };

    saveButtonStyle = {

      fontSize: "1rem",
      backgroundColor: saveButtonBgColor,
      border: saveButtonBgColor + " solid",
      margin: "36px 12px 36px 12px",
      padding: "8px",
      borderRadius: "4px",
      color: "#333333",
      fontWeight: "500",

    };

    loadButtonStyle = {

      fontSize: "1rem",
      backgroundColor: loadButtonBgColor,
      border: loadButtonBgColor + " solid",
      margin: "36px 0px 36px 12px",
      padding: "8px",
      borderRadius: "4px",
      color: "#333333",
      fontWeight: "500",

    };

    textAreaStyle = {
      fontSize: "1rem",
      fontFamily: "helvetica",
      padding: "3px",
      backgroundColor: "#f7f7f7",
      border: "1px solid #333333",
      borderRadius: "3px",
      width: "100%",
      height: "200px"
    };

    dropDownStyle = {

      fontSize: "1rem",
      padding: "3px",
      flex: 1,
      backgroundColor: "#f7f7f7",
      color: "#333333",
      borderRadius: "3px",

    };

    inputStyle = {

      fontSize: "1rem",
      padding: "3px",
      backgroundColor: "#f7f7f7",
      border: "1px solid #333333",
      borderRadius: "3px",
      marginBottom: "2px",

    };

  } else if(colorScheme == "dark") {

    setButtonStyle = {

      fontSize: "1rem",
      backgroundColor: bgColor,
      border: bgColor + " solid",
      margin: "36px 12px 36px 0px",
      padding: "8px",
      borderRadius: "4px",
      color: "#f7f7f7",
      fontWeight: "500",

    };

    saveButtonStyle = {

      fontSize: "1rem",
      backgroundColor: saveButtonBgColor,
      border: saveButtonBgColor + " solid",
      margin: "36px 12px 36px 12px",
      padding: "8px",
      borderRadius: "4px",
      color: "#f7f7f7",
      fontWeight: "500",

    };

    loadButtonStyle = {

      fontSize: "1rem",
      backgroundColor: loadButtonBgColor,
      border: loadButtonBgColor + " solid",
      margin: "36px 0px 36px 12px",
      padding: "8px",
      borderRadius: "4px",
      color: "#f7f7f7",
      fontWeight: "500",

    };

    textAreaStyle = {
      fontSize: "1rem",
      fontFamily: "helvetica",
      padding: "3px",
      backgroundColor: "#4d4d4d",
      border: "1px solid #4d4d4d",
      borderRadius: "3px",
      width: "100%",
      height: "200px"
    };

    dropDownStyle = {

      fontSize: "1rem",
      padding: "3px",
      flex: 1,
      backgroundColor: "#4d4d4d",
      color: "#f7f7f7",
      borderRadius: "3px",

    };
    
    inputStyle = {

      fontSize: "1rem",
      padding: "3px",
      backgroundColor: "#4d4d4d",
      color: "#f7f7f7",
      border: "1px solid #4d4d4d",
      borderRadius: "3px",
      marginBottom: "2px",

    };

  }

  const labelStyle = {
    fontSize: "1.35rem",
    paddingRight: "12px",
    marginBottom: "10px",
    fontWeight: "500",
  }

  const statusStyle = {
    fontSize: "0.8rem", 
    padding: "5px",
    borderTop: "0.5px solid",
  }
  
  ///////////////////////////////////////////////////////////////////

  ///////////////////////// HTML PANEL //////////////////////////////

  return (
    <div style={{ padding: "1rem", 
                  scrollBehavior: "smooth", 
                  maxHeight:"100%", 
                  overflowY: "scroll", 
                  fontFamily: "helvetica", 
                  fontSize: "1rem",
                  }}>
      <h1>ROS2 Parameter Extension</h1>
      <label style={labelStyle}>Node:</label>
      <select
        value={node}
        onChange={(event) => { node = event.target.value; updateParamList(); }}
        style={dropDownStyle}
        >
        <option selected hidden>Select a Node</option>
        {(nodeList ?? []).map((node) => (
          <option key={node} value={node}>{node}</option>
        ))}
      </select>

      <form>
        <button 
          style={setButtonStyle} 
          onMouseEnter={() => setBgColor("#8f8f8f")} 
          onMouseLeave={() => colorScheme == "dark" ? setBgColor("#4d4d4d"): setBgColor("#d6d6d6")} 
          onClick={setParam} 
          type="reset">
            Set Parameters
        </button>

        <button 
          style={saveButtonStyle } 
          id="save"
          onMouseEnter={() => setSaveButtonBgColor("#8f8f8f")} 
          onMouseLeave={() => colorScheme == "dark" ? setSaveButtonBgColor("#4d4d4d"): setSaveButtonBgColor("#d6d6d6")} 
          onClick={saveParamsToFile}
          type="button">
            Save
        </button>

        <label 
          style={loadButtonStyle} 
          onMouseEnter={() => setLoadButtonBgColor("#8f8f8f")} 
          onMouseLeave={() => colorScheme == "dark" ? setLoadButtonBgColor("#4d4d4d"): setLoadButtonBgColor("#d6d6d6")} 
          >
          <input type="file" style={{display: "none"}} onChange={(event) => {loadFile(event.target.files)}}/>
            Load
        </label>
        <br/>

        <label style={labelStyle}>Save to YAML</label>
        <br/>

        <textarea style={textAreaStyle} value={paramsToYaml}></textarea>
        <br/>

        <label style={labelStyle}>Parameter List</label>
        <br/>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 0.75fr 1fr 0.75fr", rowGap: "0.2rem",  }}>
          <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px" , }}>Parameter</b>
          <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px" }}>Type</b>
          <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px" }}>Value</b>
          <b style={{ borderBottom: "1px solid", padding: "2px", marginBottom: "3px" }}>New Value</b>
          
          {(paramList ?? []).map((result) => (
            <>
              <div style={{margin: "0px 4px 0px 4px"}} key={result.name}>{result.name}:</div>
              <div style={{margin: "0px 4px 0px 4px"}}>{getType(result.value)}</div>
              <div style={{margin: "0px 4px 0px 4px"}}>{getParameterValue(result.value)}</div>
              <div style={{margin: "0px 4px 0px 4px"}}> 
                {createInputBox(result)}
                </div>  
            </>
          ))}
        </div>
      </form>
      <p style={statusStyle}>status: {status}</p>

    </div>
  );

  ///////////////////////////////////////////////////////////////////

}

export function initParamsPanel(context: PanelExtensionContext) {
  ReactDOM.render(<ExamplePanel context={context} />, context.panelElement);
}