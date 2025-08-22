import os
import importlib.util
import yaml

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))



def import_from_first_available(yaml_path,target_globals=None):
    load_dict = {}
    """
    from a YAML file, import the first available Python file and update the global namespace.
    
    :param yaml_path: Path to the YAML file containing dependencies.
    """
    with open(yaml_path, 'r') as f:
        file_deps = yaml.safe_load(f)
    
    for dep in file_deps.keys():
        file_paths_dict = file_deps[dep]
    
        if not isinstance(file_paths_dict, dict):
            raise ValueError("The value for each key in the YAML file must be a dictionary.")
        
        for file_name in file_paths_dict.keys():
            if dep in load_dict.keys():
                break
            file_path = os.path.join(SCRIPT_DIR,file_name + ".py")
            if not os.path.exists(file_path):
                print(f"file not exists: {file_path}")
                continue
                
            if not file_path.endswith('.py'):
                print(f"not python skip: {file_path}")
                continue
                
            module_name = os.path.splitext(os.path.basename(file_path))[0]
            # spec = importlib.util.spec_from_file_location(module_name, file_path)
            # module = importlib.util.module_from_spec(spec)
            # print(module, spec, module_name)
            module = importlib.import_module(module_name)
            
            try:
                print("prepare namespace")
                print("global keys:", globals().keys())
                print("module keys:", vars(module).keys())
                import_names = {}
                less_names=  []
                for name_to_import in file_paths_dict[file_name]:
                    if name_to_import not in vars(module).keys():
                        less_names.append(name_to_import)
                        continue
                    import_names.update({name_to_import: vars(module)[file_paths_dict[file_name]]})
                if len(import_names) != len(file_paths_dict[file_name]):
                    print(f"{file_name} module not have {less_names}")
                    continue
                globals().update(import_names)
                if target_globals is not None:
                    target_globals.update(import_names)
                load_dict[dep] = [file_name,import_names]
                print("update finish", load_dict)
            except Exception as e:
                print(f"load failed {file_path}: {str(e)}")
                continue
        
        # all are not imported
        if dep not in load_dict.keys():
            raise ImportError("None of the specified files could be imported.")
    return load_dict

# example usage
if __name__ == "__main__":
    try:
        load_dict = import_from_first_available("dep.yml")
        print(globals().keys())
        print(load_dict)
        load_dict["hello"][2]()
    except Exception as e:
        print(f"error: {str(e)}")