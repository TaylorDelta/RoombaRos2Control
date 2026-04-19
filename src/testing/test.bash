usage="setup-controller-package.bash FILE_NAME [CLASS_NAME]"

# Load Framework defines
script_own_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
source $script_own_dir/../../setup.bash

check_and_set_ros_distro_and_version "${ROS_DISTRO}"

FILE_NAME=$1
if [ -z "$1" ]; then
  print_and_exit "You should provide the file name! Nothing to do 😯" "$usage"
fi
if [ -f src/$FILE_NAME.cpp ]; then
  print_and_exit "ERROR:The file '$FILE_NAME' already exist! 😱!" "$usage"
fi

if [ ! -f "package.xml" ]; then
  print_and_exit "ERROR: 'package.xml' not found. You should execute this script at the top level of your package folder. Nothing to do 😯" "$usage"
fi
PKG_NAME="$(grep -Po '(?<=<name>).*?(?=</name>)' package.xml | sed -e 's/[[:space:]]//g')"

echo ""  # Adds empty line

CLASS_NAME=$2
if [ -z "$2" ]; then
  delimiter='_'
  s="$FILE_NAME$delimiter"
  CLASS_NAME=""
  while [[ $s ]]; do
    part="${s%%"$delimiter"*}"
    s=${s#*"$delimiter"}
    CLASS_NAME="$CLASS_NAME${part^}"
  done
  echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}ClassName guessed from the '$FILE_NAME': '$CLASS_NAME'. Is this correct? If not, provide it as the second parameter.${TERMINAL_COLOR_NC}"
fi

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Which license-header do you want to use? [1]"
echo "(0) None"
echo "(1) Apache-2.0)"
echo "(2) Proprietary"
echo -n -e "${TERMINAL_COLOR_NC}"
read choice
choice=${choice:="1"}

if [ "$choice" != 0 ]; then
  echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Insert your company or personal name (copyright): ${TERMINAL_COLOR_NC}"
  read NAME_ON_LICENSE
  NAME_ON_LICENSE=${NAME_ON_LICENSE=""}
  YEAR_ON_LICENSE=$(date +%Y)
fi

LICENSE_HEADER=""
case "$choice" in
"1")
  LICENSE_HEADER="$LICENSE_TEMPLATES/default_cpp.txt"
  ;;
"2")
  LICENSE_HEADER="$LICENSE_TEMPLATES/propriatery_company_cpp.txt"
esac

echo -n -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Is package already configured (is in there a working controller already)? (yes/no) [no] ${TERMINAL_COLOR_NC}"
read package_configured
package_configured=${package_configured:="no"}

echo -e "${TERMINAL_COLOR_USER_INPUT_DECISION}Do you want to setup a 'normal' or 'chainable' controller? [1]"
echo "(1) normal (single-level control)"
echo "(2) chainable"
echo -n -e "${TERMINAL_COLOR_NC}"
read choice
choice=${choice:="1"}

CONTROLLER_TYPE=""
case "$choice" in
"1")
  CONTROLLER_TYPE="normal (single-level control)"
  ;;
"2")
  CONTROLLER_TYPE="chainable"
esac

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}ATTENTION: Setting up ros2_control controller files with following parameters: file name '$FILE_NAME', class '$CLASS_NAME', package/namespace '$PKG_NAME', type '$CONTROLLER_TYPE'. Those will be placed in folder '`pwd`'.${TERMINAL_COLOR_NC}"
echo ""
echo -e "${TERMINAL_COLOR_USER_CONFIRMATION}If correct press <ENTER>, otherwise <CTRL>+C and start the script again from the package folder and/or with correct controller name.${TERMINAL_COLOR_NC}"
read

# Add folders if deleted
ADD_FOLDERS=("include/$PKG_NAME" "src")

for FOLDER in "${ADD_FOLDERS[@]}"; do
    mkdir -p $FOLDER
done

# Set file constants
CTRL_HPP="include/$PKG_NAME/$FILE_NAME.hpp"
CTRL_CPP="src/$FILE_NAME.cpp"
CTRL_PARAMS_YAML="src/$FILE_NAME.yaml"
CTRL_VALIDATE_PARAMS_HPP="include/$PKG_NAME/validate_${FILE_NAME}_parameters.hpp"
PLUGIN_XML="$PKG_NAME.xml"

if [[ "$CONTROLLER_TYPE" == "chainable" ]]; then
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/dummy_chainable_controller.hpp $CTRL_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_chainable_controller.cpp $CTRL_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.yaml $CTRL_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/validate_dummy_controller_parameters.hpp $CTRL_VALIDATE_PARAMS_HPP
  cat $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_chainable_controller_pluginlib.xml >> $PLUGIN_XML
else
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/dummy_controller.hpp $CTRL_HPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.cpp $CTRL_CPP
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller.yaml $CTRL_PARAMS_YAML
  cp -n $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_package_namespace/validate_dummy_controller_parameters.hpp $CTRL_VALIDATE_PARAMS_HPP
  cat $ROS2_CONTROL_CONTROLLER_TEMPLATES/dummy_controller_pluginlib.xml >> $PLUGIN_XML
fi

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files copied.${TERMINAL_COLOR_NC}"

# Add license header to the files
FILES_TO_LICENSE=("$CTRL_HPP" "$CTRL_CPP" "$CTRL_VALIDATE_PARAMS_HPP")
if [[ "$package_configured" == "no" ]]; then
  FILES_TO_LICENSE+=("$VC_H")
fi
# TMP_FILE=".f_tmp"
# if [[ "$LICENSE_HEADER" != "" ]]; then
#   touch $TMP_FILE
#   for FILE_TO_LIC in "${FILES_TO_LICENSE[@]}"; do
#     cat $LICENSE_HEADER > $TMP_FILE
#     sed "1,13d" $FILE_TO_LIC >> $TMP_FILE
#     mv $TMP_FILE $FILE_TO_LIC
#     sed -i "s/\\\$YEAR\\\$/${YEAR_ON_LICENSE}/g" $FILE_TO_LIC
#     sed -i "s/\\\$NAME_ON_LICENSE\\\$/${NAME_ON_LICENSE}/g" $FILE_TO_LIC
#   done
# fi

# sed all needed files
FILES_TO_SED=("${FILES_TO_LICENSE[@]}")
FILES_TO_SED+=("$PLUGIN_XML" "$CTRL_PARAMS_YAML")

for SED_FILE in "${FILES_TO_SED[@]}"; do
  sed -i "s/TEMPLATES__ROS2_CONTROL__CONTROLLER__DUMMY_PACKAGE_NAMESPACE/${PKG_NAME^^}/g" $SED_FILE
  sed -i "s/dummy_package_namespace/${PKG_NAME}/g" $SED_FILE
  sed -i "s/dummy_controller/${FILE_NAME}/g" $SED_FILE
  sed -i "s/dummy_chainable_controller/${FILE_NAME}/g" $SED_FILE
  sed -i "s/DUMMY_CONTROLLER/${FILE_NAME^^}/g" $SED_FILE
  sed -i "s/DUMMY_CHAINABLE_CONTROLLER/${FILE_NAME^^}/g" $SED_FILE
  sed -i "s/DummyClassName/${CLASS_NAME}/g" $SED_FILE
  sed -i "s/dummy_interface_type/${INTERFACE_TYPE}/g" $SED_FILE
  sed -i "s/Dummy_Interface_Type/${INTERFACE_TYPE^}/g" $SED_FILE
done

# CMakeLists.txt: Remove comments if there any and add library
DEL_STRINGS=("# uncomment the following" "# further" "# find_package(<dependency>")

for DEL_STR in "${DEL_STRINGS[@]}"; do
  sed -i "/$DEL_STR/d" CMakeLists.txt
done

TMP_FILE=".f_tmp"
touch $TMP_FILE

# manage dependencies here
if [[ "$package_configured" == "no" ]]; then

  TEST_LINE=`awk '$1 == "find_package(ament_cmake" { print NR }' CMakeLists.txt`
  let CUT_LINE=$TEST_LINE-1
  head -$CUT_LINE CMakeLists.txt >> $TMP_FILE

  echo "set(THIS_PACKAGE_INCLUDE_DEPENDS" >> $TMP_FILE
  echo "  control_msgs" >> $TMP_FILE
  echo "  controller_interface" >> $TMP_FILE
  echo "  hardware_interface" >> $TMP_FILE
  echo "  pluginlib" >> $TMP_FILE
  echo "  rclcpp" >> $TMP_FILE
  echo "  rclcpp_lifecycle" >> $TMP_FILE
  echo "  realtime_tools" >> $TMP_FILE
  echo "  std_srvs" >> $TMP_FILE
  echo ")" >> $TMP_FILE

  echo "" >> $TMP_FILE
  echo "find_package(ament_cmake REQUIRED)" >> $TMP_FILE
  echo "find_package(generate_parameter_library REQUIRED)" >> $TMP_FILE
  echo "foreach(Dependency IN ITEMS \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
  echo "  find_package(\${Dependency} REQUIRED)" >> $TMP_FILE
  echo "endforeach()" >> $TMP_FILE
  echo "" >> $TMP_FILE
fi

# Add Plugin library related stuff
echo "# Add ${FILE_NAME} library related compile commands" >> $TMP_FILE
echo "generate_parameter_library(${FILE_NAME}_parameters" >> $TMP_FILE
echo "  ${CTRL_PARAMS_YAML}" >> $TMP_FILE
echo "  ${CTRL_VALIDATE_PARAMS_HPP}" >> $TMP_FILE
echo ")" >> $TMP_FILE

echo "add_library(" >> $TMP_FILE
echo "  $FILE_NAME" >> $TMP_FILE
echo "  SHARED" >> $TMP_FILE
echo "  $CTRL_CPP" >> $TMP_FILE
echo ")" >> $TMP_FILE
echo "target_include_directories($FILE_NAME PUBLIC" >> $TMP_FILE
echo '  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"' >> $TMP_FILE
echo '  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")' >> $TMP_FILE
echo "target_link_libraries($FILE_NAME ${FILE_NAME}_parameters)" >> $TMP_FILE
echo "ament_target_dependencies($FILE_NAME \${THIS_PACKAGE_INCLUDE_DEPENDS})" >> $TMP_FILE
echo "target_compile_definitions($FILE_NAME PRIVATE \"${FILE_NAME^^}_BUILDING_DLL\")" >> $TMP_FILE

if [[ "$package_configured" == "no" ]]; then

  echo "" >> $TMP_FILE
  echo "pluginlib_export_plugin_description_file(" >> $TMP_FILE
  echo "  controller_interface $PLUGIN_XML)" >> $TMP_FILE

  echo "" >> $TMP_FILE
  echo "install(" >> $TMP_FILE
  echo "  TARGETS" >> $TMP_FILE
  echo "  $FILE_NAME" >> $TMP_FILE
  echo "  RUNTIME DESTINATION bin" >> $TMP_FILE
  echo "  ARCHIVE DESTINATION lib" >> $TMP_FILE
  echo "  LIBRARY DESTINATION lib" >> $TMP_FILE
  echo ")" >> $TMP_FILE

  if [[ ! `grep -q "DIRECTORY include/" $TMP_FILE` ]]; then
    echo "" >> $TMP_FILE
    echo "install(" >> $TMP_FILE
    echo "  DIRECTORY include/" >> $TMP_FILE
    echo '  DESTINATION include/${PROJECT_NAME}' >> $TMP_FILE
    echo ")" >> $TMP_FILE
  fi
fi

echo ""  >> $TMP_FILE

mv $TMP_FILE CMakeLists.txt

# manage dependencies in package.xml
if [[ "$package_configured" == "no" ]]; then

  append_to_string="<buildtool_depend>ament_cmake<\/buildtool_depend>"
  sed -i "s/$append_to_string/$append_to_string\\n\\n  <build_depend>generate_parameter_library<\/build_depend>/g" package.xml

  DEP_PKGS=("std_srvs" "realtime_tools" "rclcpp_lifecycle" "rclcpp" "pluginlib" "hardware_interface" "controller_interface" "control_msgs")

  for DEP_PKG in "${DEP_PKGS[@]}"; do
    if `grep -q "<depend>${DEP_PKG}</depend>" package.xml`; then
      echo "'$DEP_PKG' is already listed in package.xml"
    else
      append_to_string="<build_depend>generate_parameter_library<\/build_depend>"
      sed -i "s/$append_to_string/$append_to_string\\n\\n  <depend>${DEP_PKG}<\/depend>/g" package.xml
    fi
  done
fi

# Remove lint dependencies
sed -i "/ament_lint_auto_find_test_dependencies()/d" CMakeLists.txt
sed -i "/<test_depend>ament_lint_common<\/test_depend>/d" package.xml

# extend README with general instructions
if [ -f README.md ]; then

  echo "" >> README.md
  echo "Pluginlib-Library: $FILE_NAME" >> README.md
  echo ""
  echo "Plugin: $PKG_NAME/${CLASS_NAME} (controller_interface::ControllerInterface)" >> README.md

fi

echo -e "${TERMINAL_COLOR_USER_NOTICE}Template files were adjusted.${TERMINAL_COLOR_NC}"

git add .

# Compile and add new package the to the path
compile_and_source_package $PKG_NAME "yes"

echo ""
echo -e "${TERMINAL_COLOR_USER_NOTICE}FINISHED: Your package is set.${TERMINAL_COLOR_NC}"
