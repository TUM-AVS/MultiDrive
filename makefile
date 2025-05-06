# Use motage instead of convert to create a grid 
# https://stackoverflow.com/questions/23852902/appending-images-using-imagemagick-convert
# https://stackoverflow.com/questions/20737061/merge-images-side-by-side-horizontally


SHELL := /bin/bash
PYTHON=.venv/bin/python
VEHICLE_MODEL ?= car
OUTPUT_FOLDER=experiments-$(VEHICLE_MODEL)
SED=/usr/bin/sed
UNAME := $(shell uname)
# Override if on Mac Os. Assume that you have installed Gnu SED (gsed)
ifeq ($(UNAME),Darwin)
SED := $(shell echo /usr/local/bin/gsed)
endif

# Find all the scenarios that can be executed in the "scenarios" folder and use them to build dynamically all the make targets
scenario_file_names := $(shell find scenarios -iname "*.xml" -exec basename {} \; | sed -e "s|.xml||g")

PHONY: list
list:
	@LC_ALL=C $(MAKE) -pRrq -f $(firstword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/(^|\n)# Files(\n|$$)/,/(^|\n)# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | grep -E -v -e '^[^[:alnum:]]' -e '^$@$$'
# IMPORTANT: The line above must be indented by (at least one) 
#            *actual TAB character* - *spaces* do *not* work.

list-scenario-names:
	@echo $(scenario_file_names)

##########################################################


# Generates the target for each scenario to
# Simulate and Cosimulate with BNG (so not Cosimulate with Fault Injection and Resimulate here)
# Plot them, Pad the plot, Merge the plots, Animate the plots
# Compare and plot error metrics
define make-simulation-targets-for-scenarios

clean-$1:
	@rm -rfv $(OUTPUT_FOLDER)/$1

simulate-$1: $(OUTPUT_FOLDER)/$1/simulated_scenario.xml
	@echo "Done Simulating"

$(OUTPUT_FOLDER)/$1/scenario_to_simulate.xml $(OUTPUT_FOLDER)/$1/simulated_scenario.xml:
	@echo "Setting up output folder $(OUTPUT_FOLDER)/$1"
	@rm -rf $(OUTPUT_FOLDER)/$1/00001
	@mkdir -p $(OUTPUT_FOLDER)/$1
	$(eval SCENARIO_FILE=$(shell find scenarios -iname "*.xml" | grep $1.xml))
	$(PYTHON) cli.py \
		--output-folder $(OUTPUT_FOLDER)/$1 --scenario-name 00001 \
		simulate \
			--scenario-file $(SCENARIO_FILE) 2>&1 | tee $(OUTPUT_FOLDER)/$1/simulation.log


cosimulate_with_bng-$1: $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml
	@echo "Done Cosimulating with BeamNG"

$(OUTPUT_FOLDER)/$1/scenario_to_cosimulate_with_bng.xml $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml: 
	@echo "Setting up output folder $(OUTPUT_FOLDER)/$1"
	@rm -rf $(OUTPUT_FOLDER)/$1/00002
	@mkdir -p $(OUTPUT_FOLDER)/$1
	$(eval SCENARIO_FILE=$(shell find scenarios -iname "*.xml" | grep $1.xml))
	$(PYTHON) cli.py \
		--output-folder $(OUTPUT_FOLDER)/$1 --scenario-name 00002 \
		cosimulate-with-beamng \
			--vehicle-model $(VEHICLE_MODEL) \
			--scenario-file $(SCENARIO_FILE) 2>&1 | tee $(OUTPUT_FOLDER)/$1/cosimulation_with_bng.log

endef

# Generating the targets for executing the scenarios based on the files in the scenario folder
$(foreach scenario_id,$(scenario_file_names),$(eval $(call make-simulation-targets-for-scenarios,$(scenario_id))))


define make-plotting-targets-for-scenarios

plot-simulated-$1: $(OUTPUT_FOLDER)/$1/simulated_scenario.xml $(OUTPUT_FOLDER)/$1/simulated-plots/.done
	@echo "Done Plotting Simulation"

$(OUTPUT_FOLDER)/$1/simulated-plots/.done: $(OUTPUT_FOLDER)/$1/scenario_to_simulate.xml $(OUTPUT_FOLDER)/$1/simulated_scenario.xml
	@echo "Plotting $$@"
	$(PYTHON) cli.py \
		--output-folder $(OUTPUT_FOLDER)/$1/simulated-plots \
		plot-simulation \
			--label CommonRoad_Simulation \
			--original-scenario $(OUTPUT_FOLDER)/$1/scenario_to_simulate.xml \
			--executed-scenario $(OUTPUT_FOLDER)/$1/simulated_scenario.xml
	@touch $(OUTPUT_FOLDER)/$1/simulated-plots/.done


plot-cosimulated_with_bng-$1: $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done
	@echo "Done Plotting Cosimulation with BeamNG"

$(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done: $(OUTPUT_FOLDER)/$1/scenario_to_cosimulate_with_bng.xml $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml
	@echo "Plotting $$@"
	$(PYTHON) cli.py \
		--output-folder $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots \
		plot-simulation \
			--label BeamNG_Cosimulation \
			--original-scenario $(OUTPUT_FOLDER)/$1/scenario_to_cosimulate_with_bng.xml \
			--executed-scenario $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml
	@touch $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done

endef


# Generate the plotting targets
$(foreach scenario_id,$(scenario_file_names),$(eval $(call make-plotting-targets-for-scenarios,$(scenario_id))))

define make-animation-targets-for-scenarios

pad-simulated-plots-$1: $(OUTPUT_FOLDER)/$1/simulated-plots/.padded
	@echo "Done padding"

$(OUTPUT_FOLDER)/$1/simulated-plots/.padded: $(OUTPUT_FOLDER)/$1/simulated-plots/.done
$(OUTPUT_FOLDER)/$1/simulated-plots/.padded: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done
	$$(eval LAST_SIMULATED_FRAME=$$(shell find $(OUTPUT_FOLDER)/$1/simulated-plots -iname "*.png" | sort | tail -1 | sed -e "s|.*_\(.*\).png|\1|")) 
	$$(eval LAST_COSIMULATED_with_bng_FRAME=$$(shell find $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots -iname "*.png" | sort | tail -1 | sed -e "s|.*_\(.*\).png|\1|"))

	$$(eval MAX_FRAME=$$(shell echo $$(LAST_SIMULATED_FRAME) $$(LAST_COSIMULATED_with_bng_FRAME) | tr " " "\n" | sort -n | tail -1))
		
	@if [ ! $$(LAST_SIMULATED_FRAME) -eq $$(MAX_FRAME) ]; then \
		for i in $$$$(seq -w $$(LAST_SIMULATED_FRAME) $$(MAX_FRAME)); do \
			cp -v $(OUTPUT_FOLDER)/$1/simulated-plots/frame_$$(LAST_SIMULATED_FRAME).png $(OUTPUT_FOLDER)/$1/simulated-plots/frame_$$$$i.png; \
		done; \
	fi

	@touch $(OUTPUT_FOLDER)/$1/simulated-plots/.padded


pad-cosimulated_with_bng-plots-$1: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.padded
	@echo "Done padding"

$(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.padded: $(OUTPUT_FOLDER)/$1/simulated-plots/.done
$(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.padded: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done
	$$(eval LAST_SIMULATED_FRAME=$$(shell find $(OUTPUT_FOLDER)/$1/simulated-plots -iname "*.png" | sort | tail -1 | sed -e "s|.*_\(.*\).png|\1|")) 
	$$(eval LAST_COSIMULATED_with_bng_FRAME=$$(shell find $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots -iname "*.png" | sort | tail -1 | sed -e "s|.*_\(.*\).png|\1|"))

	$$(eval MAX_FRAME=$$(shell echo $$(LAST_SIMULATED_FRAME) $$(LAST_COSIMULATED_with_bng_FRAME) | tr " " "\n" | sort -n | tail -1))

	@if [ ! $$(LAST_COSIMULATED_with_bng_FRAME) -eq $$(MAX_FRAME) ]; then \
		for i in $$$$(seq -w $$(LAST_COSIMULATED_with_bng_FRAME) $$(MAX_FRAME)); do \
			cp -v $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/frame_$$(LAST_COSIMULATED_with_bng_FRAME).png $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/frame_$$$$i.png; \
		done; \
	fi
	
	@touch $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.padded


merge-plots-$1: $(OUTPUT_FOLDER)/$1/merged-plots/.done
	@echo "Done Merging plots"
	
$(OUTPUT_FOLDER)/$1/merged-plots/.done: $(OUTPUT_FOLDER)/$1/simulated-plots/.padded
$(OUTPUT_FOLDER)/$1/merged-plots/.done: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.padded
	$$(eval MAX_FRAME=$$(shell find $(OUTPUT_FOLDER)/$1/simulated-plots -iname "*.png" | sort | tail -1 | sed -e "s|.*_\(.*\).png|\1|")) 
	@rm -rfv $(OUTPUT_FOLDER)/$1/merged-plots
	@mkdir -p $(OUTPUT_FOLDER)/$1/merged-plots
	@echo "Merging plots for scenario $1"
	@for i in $$$$(seq -w 00001 $$(MAX_FRAME)); do \
		convert +append \
			$(OUTPUT_FOLDER)/$1/simulated-plots/frame_$$$${i}.png \
			$(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/frame_$$$${i}.png \
			$(OUTPUT_FOLDER)/$1/merged-plots/frame_$$$${i}.png; \
	done
	
	@touch $(OUTPUT_FOLDER)/$1/merged-plots/.done


animate-simulated-$1: $(OUTPUT_FOLDER)/$1/simulated-scenario.gif

$(OUTPUT_FOLDER)/$1/simulated-scenario.gif: $(OUTPUT_FOLDER)/$1/simulated-plots/.done
	@echo "Animating simulated scenario $1"
	gifski --repeat 1 --output $(OUTPUT_FOLDER)/$1/simulated-scenario.gif $(OUTPUT_FOLDER)/$1/simulated-plots/frame_*.png

animate-cosimulated_with_bng-$1: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-scenario.gif

$(OUTPUT_FOLDER)/$1/cosimulated_with_bng-scenario.gif: $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/.done
	@echo "Animating cosimulated scenario $1"
	gifski --repeat 1 --output $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-scenario.gif $(OUTPUT_FOLDER)/$1/cosimulated_with_bng-plots/frame_*.png

animate-$1: $(OUTPUT_FOLDER)/$1/scenario.gif

$(OUTPUT_FOLDER)/$1/scenario.gif: $(OUTPUT_FOLDER)/$1/merged-plots/.done
	@echo "Animating scenario $1"
	gifski --repeat 1 --output $(OUTPUT_FOLDER)/$1/scenario.gif $(OUTPUT_FOLDER)/$1/merged-plots/frame_*.png


all-$1: analyse-$1 animate-$1
	@echo "Done with scenario $1"

all: analyse-$1 animate-$1 

endef

$(foreach scenario_id,$(scenario_file_names),$(eval $(call make-animation-targets-for-scenarios,$(scenario_id))))

define make-data-analysis-targets-for-scenarios

$(OUTPUT_FOLDER)/$1/plots/overall.png $(OUTPUT_FOLDER)/$1/plots/errors.png: $(OUTPUT_FOLDER)/$1/simulated_scenario.xml
$(OUTPUT_FOLDER)/$1/plots/overall.png $(OUTPUT_FOLDER)/$1/plots/errors.png: $(OUTPUT_FOLDER)/$1/cosimulated_scenario_with_bng.xml
	$(PYTHON) cli.py \
		--output-folder $(OUTPUT_FOLDER)/$1 \
		compare \
			--executed-scenario-folder $(OUTPUT_FOLDER)/$1

$(OUTPUT_FOLDER)/$1/simulation_vs_cosimulation_with_bng.csv: $(OUTPUT_FOLDER)/$1/plots/overall.png
$(OUTPUT_FOLDER)/$1/simulation_vs_cosimulation_with_bng.csv: $(OUTPUT_FOLDER)/$1/plots/errors.png

analyse-$1: $(OUTPUT_FOLDER)/$1/plots/overall.png

analyse-$1: $(OUTPUT_FOLDER)/$1/plots/errors.png

analyse-$1: $(OUTPUT_FOLDER)/$1/simulation_vs_cosimulation_with_bng.csv
	@echo "Done"

endef

$(foreach scenario_id,$(scenario_file_names),$(eval $(call make-data-analysis-targets-for-scenarios,$(scenario_id))))