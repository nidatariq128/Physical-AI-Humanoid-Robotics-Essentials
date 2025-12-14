
# Data Model: AI Robotics Book

## Core Entities

### Book Section
- **name**: string (required) - The section or chapter name
- **module**: enum (foundations|ros2|simulation|perception|vla|capstone|hardware) - The module this section belongs to
- **content**: string (required) - The markdown content of the section
- **word_count**: integer - Number of words in the section
- **citations**: array of Citation - References used in this section
- **learning_objectives**: array of string - What readers should learn from this section
- **prerequisites**: array of string - Knowledge required before reading this section
- **validation_status**: enum (draft|reviewed|validated|published) - The review status

### Citation
- **id**: string (required) - Unique identifier for the citation
- **type**: enum (journal|conference|book|documentation|standard|web) - Type of source
- **title**: string (required) - Title of the source
- **authors**: array of string - Authors of the source
- **year**: integer - Publication year
- **publisher**: string - Publisher or venue
- **url**: string - URL if available
- **doi**: string - DOI if available
- **peer_reviewed**: boolean - Whether this is peer-reviewed literature
- **apa_citation**: string (required) - Properly formatted APA citation
- **access_date**: date - When this source was accessed

### Module
- **id**: string (required) - Unique identifier for the module
- **name**: string (required) - Display name of the module
- **description**: string - Brief description of the module
- **order**: integer (required) - Order in the 13-week progression
- **sections**: array of BookSection - Sections that belong to this module
- **learning_outcomes**: array of string - What readers should achieve after completing this module
- **prerequisites**: array of string - What knowledge is required before this module
- **estimated_duration**: integer - Estimated time in hours to complete

### Weekly Milestone
- **week_number**: integer (required) - Week number in the 13-week progression
- **module_id**: string (required) - Module this milestone belongs to
- **topics**: array of string - Topics to be covered in this week
- **learning_objectives**: array of string - Objectives for this week
- **assignments**: array of string - Practical assignments for this week
- **dependencies**: array of string - What must be completed before this week

### Hardware Configuration
- **tier**: enum (proxy|minibot|premium) - Hardware tier classification
- **name**: string (required) - Name of the configuration
- **description**: string - Description of the hardware setup
- **components**: array of HardwareComponent - Components in this configuration
- **estimated_cost**: integer - Estimated cost in USD
- **suitable_for**: array of string - What applications this configuration supports
- **compatibility**: array of string - Compatible software platforms

### Hardware Component
- **name**: string (required) - Name of the component
- **type**: enum (processor|sensor|actuator|chassis|battery|communication) - Type of component
- **specifications**: object - Technical specifications
- **manufacturer**: string - Component manufacturer
- **model**: string - Model number
- **cost**: integer - Cost in USD
- **compatibility**: array of string - Compatible platforms
- **alternatives**: array of string - Alternative components

### Capstone Project
- **name**: string (required) - Name of the capstone project
- **description**: string - Detailed description of the project
- **requirements**: array of string - Technical requirements
- **subsystems**: array of string (speech_recognition|planning|navigation|manipulation) - Required subsystems
- **evaluation_criteria**: array of string - How the project will be evaluated
- **simulated**: boolean - Whether project is simulated or real hardware
- **integration_points**: array of string - How this integrates with other modules

## Relationships

- **Module** contains many **BookSection**
- **BookSection** references many **Citation**
- **Module** connects to **Weekly Milestone**
- **Weekly Milestone** belongs to one **Module**
- **Capstone Project** integrates many **Module**
- **Hardware Configuration** contains many **Hardware Component**

## Validation Rules

1. **BookSection.word_count** must be between 0 and 1000 (to maintain section granularity)
2. **Citation.peer_reviewed** must be true for at least 50% of citations in the complete book
3. **Citation.apa_citation** must follow proper APA format
4. **Module.order** must be unique within the book
5. **Weekly Milestone.week_number** must be between 1 and 13
6. **BookSection.citations** must not be empty if section has factual claims
7. **Module.learning_outcomes** must align with specification requirements
8. **Citation.type** must be appropriate for the content domain

## State Transitions

**BookSection**:
- draft → reviewed (when peer review complete)
- reviewed → validated (when citation and fact-checking complete)
- validated → published (when integrated into final book)

**Module**:
- planned → in_progress (when first section starts development)
- in_progress → completed (when all sections validated)
- completed → integrated (when connected to other modules)